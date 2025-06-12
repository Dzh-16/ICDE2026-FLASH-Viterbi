#define _CRT_SECURE_NO_WARNINGS // FOPEN

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <windows.h>

// #define STR_LEN 5   // Length of input state sequence characters
// #define K_STATE 128 // State space size
// #define T_STATE 100 // Observation state size
#define K_STATE 3965 // State space size                        Modified to same value as K
#define T_STATE 50 // Observation state size
// #define SUPER_K_STATE     20      //State space size
// #define SUPER_T_STATE     20
// #define FST_SIZE    5       //firstN size
// #define BST_SIZE    5       //bestN size

#define ObserRouteLEN 256 // Observation path length             Modified to same value as T

#define BeamSearchWidth 128// State constraint size                        Modified to same value as Beam width

#define MAX_THREADS 7
clock_t start_time;
clock_t end_time; // COUNT TIME
LARGE_INTEGER t1, t2, tc;

typedef float ElementType; // MAX HEAP sorting element type is float, transition probability unit is double
typedef int Status;         /* Status is function type, its value is function result status code, like OK?? */

ElementType A[K_STATE][K_STATE]; // When array dimension is too large, use malloc for large memory
ElementType B[K_STATE][T_STATE];
ElementType pi[K_STATE];
int Obroute[ObserRouteLEN]; // Observation sequence

// int spaceNaiveViterbiPath[ObserRouteLEN];//space naive viterbi path
//  int snvT2[K_STATE];               // T2_table,path
//  int snvT3[K_STATE];               // Mid Table,midpath
int snvOutPutPath[ObserRouteLEN]; // output
/// Queue storage related///////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct Queue
{
    int *preNode; // Predecessor of problem path
    int *sucNode; // Successor of problem path
    int front;    // Head pointer
    int rear;     // Tail pointer
} Queue;

// Thread pool structure
typedef struct {
    HANDLE threads[MAX_THREADS]; // Thread handle array
    CRITICAL_SECTION lock;       // Critical section object for thread-safe operations
    CONDITION_VARIABLE pool_wake;  // Event object array for thread synchronization
    Queue Q;
    int active_threads;          // Number of active threads
    int task_count;
} ThreadPool;
int TotalTask;

/// Max heap related///////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct element
{
    ElementType Value;
    int State;
    int T3_State;
} element;
// typedef struct TreeNode // BST TREE NODE        Tree
// {
//     float count_score; // Records corresponding node probability value
//     int node_no;       // Records current node corresponding hidden state
//     struct TreeNode* lchild;
//     struct TreeNode* rchild;
// } TreeNode;

void insert_binary_tree(element *heap, ElementType Value, int State, int T3_State)
{
    (*heap).Value = Value;
    (*heap).State = State;
    (*heap).T3_State = T3_State;
}

void initial_heap_element(element *heap)
{
    (*heap).Value = 0;
    (*heap).State = -1;
    (*heap).T3_State = -1;
    // for (int i = 1; i < BeamSearchWidth + 1; i++)
    // {
    //     (*(heap + i)).Value = 0;
    //     (*(heap + i)).State = -1;
    //     (*(heap + i)).T3_State = -1;
    // }
}


int Find_T3_State(element **heap, int state)
{
    int total = (*heap)[0].Value; // Get total number of elements in heap

    for (int i = 1; i <= total; i++)
    {
        if ((*heap)[i].State == state)
        {
            return (*heap)[i].T3_State; // If matching state found, return corresponding T3_State value
        }
    }

    return -1; // If no matching state found, return -1
}

void print_heap_element(element **heap)
{
    for (int i = 1; i < BeamSearchWidth + 1; i++)
    {
        printf("heap element is %lf,state is %d,T3 state is %d\n", (*((*heap) + i)).Value, (*((*heap) + i)).State, (*((*heap) + i)).T3_State);
    }
}

void create_min_heap(element **heap)
{
    int total = (**heap).Value;
    /// Find last non-leaf node
    int child = 2, parent = 1;
    for (int node = total / 2; node > 0; node--)
    {
        parent = node;
        child = 2 * node;
        // int max_node = 2 * node + 1;
        element temp = *((*heap) + parent);     // Save current parent node
        // for (; child <= total; child *= 2, max_node = 2 * parent + 1)
        for (; child <= total; child *= 2)
        {
            if (child + 1 <= total && (*((*heap) + child)).Value > (*((*heap) + child + 1)).Value)       //Find smaller child node
            {
                child++;
            }
            if (temp.Value <= (*((*heap) + child)).Value)       //If parent node <= smallest child node, heap property satisfied, break loop
            { // Changed from < to <=, no swap for child nodes equal to parent
                break;
            }
            *((*heap) + parent) = *((*heap) + child);
            parent = child;
        }
        *((*heap) + parent) = temp;     //Assign value to parent node position, *heap is heap head address, plus parent position is parent address, then * gets value
    }
}

// **
//  * Replace smallest element in heap and maintain min heap property
//  * @param heap Min heap
//  * @param newValue New replacement value
//  * @param newState New state
//  */
void replace_min_heap_element(element **heap, ElementType newValue, int newState, int newT3_State)
{
    // Replace heap top element with new value
    (*heap)[1].Value = newValue;
    (*heap)[1].State = newState;
    (*heap)[1].T3_State = newT3_State;

    int total = (*heap)[0].Value; // Total element count
    int parent = 1;
    int child = 2;

    // Top-down heap adjustment
    while (child <= total)
    {
        // Find smaller of left/right child nodes
        if (child + 1 <= total && (*heap)[child].Value > (*heap)[child + 1].Value)
        {
            child++;
        }

        // If parent <= smallest child, heap property satisfied
        if ((*heap)[parent].Value <= (*heap)[child].Value)
        {
            break;
        }

        // Swap parent and child
        element temp = (*heap)[parent];
        (*heap)[parent] = (*heap)[child];
        (*heap)[child] = temp;

        parent = child;
        child *= 2;
    }
}

Status generate_state_heap(ElementType probability_i, int i, element **heap_total, int *changestate, int T3_State)
{
    element *num = *heap_total;      // Position storing tree count
    element *position = *heap_total; // Current node position in tree
    position = position + i + 1;     // positi moves back i+1 positions each time, already calculated i points
    if (i < BeamSearchWidth - 1)
    { // When tree not full
        insert_binary_tree(position, probability_i, i, T3_State);
        (*num).Value++;
        // Output result after insertion, for testing
        //  printf("total number is %d ,new element is %lf  ,state is %d  ,T3 State is%d\n",(int)((*num).Value),(*position).Value,(*position).State,(*position).T3_State);
        return 0;
    }
    else if (i == BeamSearchWidth - 1)
    { // Tree just filled
        insert_binary_tree(position, probability_i, i, T3_State);
        (*num).Value++;
        // Output result after insertion, for testing
        //  printf("\ntotal number is %d  ,new element is %lf  ,state is %d  \n",(int)((*num).Value),(*position).Value,(*position).State);
        create_min_heap(heap_total);
        // Output sorted result, for testing
        //  print_heap_element(heap_total);
        return 0;
    }
    else
    { // Tree full, and next element > min value, replace and output 1
        if (probability_i > (*heap_total)[1].Value)
        {
            // printf("\nChange:new element value is %lf state is %d T3 State is%d,Total min element is %lf  state is %d T3 State is%d \n",probability_i,i,T3_State,(*heap_total)[1].Value,(*heap_total)[1].State,(*heap_total)[1].T3_State);
            *changestate = (*heap_total)[1].State; // Replaced state
            // (*heap_total)[1].Value=probability_i;
            // (*heap_total)[1].State=i;
            // create_max_heap(heap_total);
            replace_min_heap_element(heap_total, probability_i, i, T3_State);
            // Output sorted result, for testing
            //  print_heap_element(heap_total);
            return 1;
        }
        // Tree full, and next element <= min value, no replace, output 2
        else
        {
            // printf("Keep:new element value is %lf  state is %d  ,Total min element is %lf  state is %d  \n",probability_i,i,(*heap_total)[1].Value,(*heap_total)[1].State);
            return 2;
        }
    }
}

// void chaneg_T2_i(int changestate,int changgelabel,int i,int statei_value){

//     if (!changgelabel){         //=0 means not full yet, can directly assign T2
//         snvT2[i] =  statei_value;
//         // printf("input state : %d , Arc: %d -> %d \n",insert,statei_value,i);
//         }
//     else if(changgelabel==1){   //=1 means full and need to replace state i, corresponding state is statei_value, changgelabel is replaced state
//         snvT2[i] =  statei_value;
//         snvT2[changestate] =  -1;
//         printf("change stae %d value to %d ,input state %d value to %d\n",snvT2[changestate],changestate,snvT2[i],i);
//         }
//     else{                       //=2 means full but no replacement needed
//         snvT2[i] =  -1;
//         printf("input stae %d value to %d\n",snvT2[i],i);
//         }
// }

// void  Change_T3_element(element **heap,int snvT3[]){
//     for (int i = 1; i<BeamSearchWidth+1; i++) {
//         snvT3[(*((*heap) + i)).State]=(*((*heap) + i)).T3_State;
//         // printf("heap element is %lf,state is %d,T3 state is %d\n",(*((*heap) + i)).Value,(*((*heap) + i)).State,(*((*heap) + i)).T3_State);}
// }
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void InitOutputpath()
{

    int i = 0;

    for (i = 0; i < ObserRouteLEN; i++)
    {

        snvOutPutPath[i] = -1;
    }
}

void InitQueue(Queue *Q) // Initialize queue
{
    // Allocate storage space for queue, for predecessor/successor queues
    Q->preNode = (int *)malloc(sizeof(int) * ObserRouteLEN); // Allocate integer storage space of observation path length for pointer variable.
    Q->sucNode = (int *)malloc(sizeof(int) * ObserRouteLEN);
    if ((Q->preNode != NULL) && (Q->sucNode != NULL))
        Q->front = Q->rear = 0;
    // Initially queue empty, head and tail pointers point to position 0
    else
    {
        printf("allocation is failed !!");
        return;
    }
}

void EnQueue(Queue *Q, int preside, int sucside) // Enqueue
{
    // Check if circular queue is full
    if (((Q->rear + 1) % ObserRouteLEN) == Q->front) // Full condition is next position of tail pointer (via modulo (Q->rear + 1) % ObserRouteLEN) equals head pointer position (Q->front).
        return;
    // Queue not full, enqueue data
    Q->preNode[Q->rear] = preside; // Predecessor boundary enqueue
    Q->sucNode[Q->rear] = sucside; // Successor boundary enqueue
    // Update tail pointer
    Q->rear = (Q->rear + 1) % ObserRouteLEN;
}

Status DeQueue(Queue *Q, int *preside, int *sucside) // Dequeue
{
    // Check if circular queue is empty
    if (Q->front == Q->rear)
        return 0;
    // Before dequeue, save predecessor and successor node values
    *preside = Q->preNode[Q->front];
    *sucside = Q->sucNode[Q->front];
    // If not empty, implement circular dequeue
    Q->front = (Q->front + 1) % ObserRouteLEN;
    return 1;
}
void ShowQueue(Queue *Q)
{
    // Traverse elements in circular queue and print data
    for (int i = Q->front; i != Q->rear; i = (i + 1) % ObserRouteLEN)
    {
        printf("(%d->%d), ", Q->preNode[i], Q->sucNode[i]);
        // This operation implements circular traversal
    }
    printf("\n");
}

int Length(Queue *Q)
{
    // Calculate difference between tail and head pointer positions
    int len = Q->rear - Q->front;
    // If positive, len is queue length; if negative, ObserRouteLEN+len is queue length
    len = (len > 0) ? len : ObserRouteLEN + len;
    return len;
}

void max_score(ElementType arr[], int arr_count, int *max_temp_no, ElementType *max_temp) // Find max probability value and index in queue
{

    if (NULL == arr)
        return;
    int max_no = 0;           // Probability value No.
    ElementType max = arr[0]; // Probability value
    for (int i = 0; i < arr_count; i++)
    {
        if (max < arr[i])
        {
            max = arr[i];
            max_no = i;
        }
    }
    *max_temp_no = max_no;
    *max_temp = max;
}

void InitMatrixA(char* str_MatrixA)
{

    int i = 0, j = 0;
    ElementType *tmpt = (ElementType *)malloc(sizeof(ElementType) * K_STATE * K_STATE);

    if (tmpt)
    {
        // ElementType tmpt[K_STATE*K_STATE]= {0};//Large array overflow
        //  FILE* fpRead = fopen("C:/Users/DELL/Documents/work/source/Nvvviter/dataset/matrixA_500plus500_fixed.txt", "rb+");//matrixA or matrixB
        // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\A.txt", "rb+"); // matrixA or matrixB After successfully opening file, fopen() returns file pointer. File pointer type is FILE
        // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\A_512.txt", "rb+"); // matrixA or matrixB After successfully opening file, fopen() returns file pointer. File pointer type is FILE
        // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\new512data\\A_512.txt", "rb+"); // matrixA or matrixB After successfully opening file, fopen() returns file pointer. File pointer type is FILE
        FILE *fpRead = fopen(str_MatrixA, "rb+"); // matrixA or matrixB After successfully opening file, fopen() returns file pointer. File pointer type is FILE

        if (fpRead == NULL)
        {
            printf("open fail errno = %d reason = %s \n", errno, strerror(errno));
            return;
        }
        int retnum = 0;
        for (i = 0; i < K_STATE * K_STATE; i++)
        {
            retnum = fscanf(fpRead, "%f", &tmpt[i]); // Use fscanf to read double type data from file and store in tmpt[i]
            // printf("%f  ",tmpt[i]);
        }
        fclose(fpRead);

        for (i = 0; i < K_STATE; i++)
        {
            for (j = 0; j < K_STATE; j++)
            {
                A[i][j] = tmpt[i * K_STATE + j];
            }
        }
    }
    free(tmpt);
}

void InitMatrixB(char* str_MatrixB )
{

    int i = 0, j = 0;
    ElementType *tmpt = (ElementType *)malloc(sizeof(ElementType) * K_STATE * T_STATE);
    if (tmpt)
    {
        // static ElementType tmpt[K_STATE*T_STATE]= {0};
        //  FILE* fpRead = fopen("C:/Users/DELL/Documents/work/source/Nvvviter/dataset/matrixB_500plus500_fixed.txt", "rb+");//matrixA or matrixB
        // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\B_512.txt", "rb+"); // matrixA or matrixB
        // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\new512data\\B_512.txt", "rb+"); // matrixA or matrixB
        FILE *fpRead = fopen(str_MatrixB, "rb+"); // matrixA or matrixB

        if (fpRead == NULL)
        {
            return;
        }
        int retnum = 0;
        for (i = 0; i < K_STATE * T_STATE; i++)
        {
            retnum = fscanf(fpRead, "%f", &tmpt[i]);
        }
        fclose(fpRead);

        for (i = 0; i < K_STATE; i++)
        {
            for (j = 0; j < T_STATE; j++)
            {
                B[i][j] = tmpt[i * T_STATE + j];
            }
        }
    }

    free(tmpt);
}

void InitMatrixPI(char* str_MatrixPI)
{
    int i, j;
    ElementType *tmpt = (ElementType *)malloc(sizeof(ElementType) * K_STATE);
    if (tmpt == NULL)
    {
        printf("malloc failed!!");
        return;
    }
    // ElementType tmpt[K_STATE]= {0};
    //  FILE* fpRead = fopen("C:/Users/DELL/Documents/work/source/Nvvviter/dataset/matrixPi500_fixed.txt", "rb+");
    // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\Pi_512.txt", "rb+");
    // FILE *fpRead = fopen("E:\\Code\\Ccode\\test\\Nvviter\\new512data\\Pi_512.txt", "rb+");
    FILE *fpRead = fopen(str_MatrixPI, "rb+");


    if (fpRead == NULL)
    {
        return;
    }
    int retnum = 0;
    for (i = 0; i < K_STATE; i++)
    {
        retnum = fscanf(fpRead, "%f", &tmpt[i]);
    }
    fclose(fpRead);
    for (j = 0; j < K_STATE; j++)
    {
        pi[j] = tmpt[j];
    }
    printf("\n");
    free(tmpt);

    //
}

void InitObRoute(char* str_ObRoute)
{
    int i, j;
    int tmpt[ObserRouteLEN] = {0};
    // FILE* fpRead3 = fopen("C:/Users/DELL/Documents/work/source/Nvvviter/dataset/obRoute5000K500.txt", "rb+");
    // FILE *fpRead3 = fopen("E:\\Code\\Ccode\\test\\Nvviter\\ob_512.txt", "rb+");
    // FILE *fpRead3 = fopen("E:\\Code\\Ccode\\test\\Nvviter\\new512data\\ob_512.txt", "rb+");
    FILE *fpRead3 = fopen(str_ObRoute, "rb+");

    if (fpRead3 == NULL)
    {
        return;
    }
    int retnum = 0;
    for (i = 0; i < ObserRouteLEN; i++)
    {
        retnum = fscanf(fpRead3, "%d", &tmpt[i]);
    }
    fclose(fpRead3);
    for (j = 0; j < ObserRouteLEN; j++)
    {
        Obroute[j] = tmpt[j];
    }
    //
}


void ViterbiNDivide(int qpreNode, int qsucNode, int N, Queue *Q)
{

    int TobsLenth = ObserRouteLEN;
    if (qsucNode - qpreNode < 2*N) // If successor-predecessor <6, end
        return;
    int i;
    ElementType scorearr[BeamSearchWidth] = {0}; // Store all probability values for finding max probability
    int T_obs = Obroute[qpreNode];                                // Initial observation sequence value, first column observation value   *Current sequence start observation value
    int midpoint[N+1], gap_length = (qsucNode - qpreNode) / N, gap_extra = (qsucNode - qpreNode) % N;
    midpoint[0] = qpreNode, midpoint[N] = qsucNode; //0 meaningless
    for(int i = 1; i < N; ++i)
    {
        midpoint[i] = midpoint[i-1]+gap_length;
        if(gap_extra > 0) gap_extra--, midpoint[i]++;
    }
    element** heap_pre_matrix = (element**)malloc(sizeof(element*)*N);
    element** heap_total_matrix = (element**)malloc(sizeof(element*)*N);
    element *heap_pre[N];
    element *heap_total[N];
    for(int i = 0; i < N; ++i)
    {
        heap_pre_matrix[i] = (element*)malloc(sizeof(element)*(BeamSearchWidth + 1));
        heap_total_matrix[i] = (element*)malloc(sizeof(element)*(BeamSearchWidth + 1));
        heap_pre[i] = heap_pre_matrix[i];
        heap_total[i] = heap_total_matrix[i];
        initial_heap_element(heap_total[i]);
        initial_heap_element(heap_pre[i]);
    }
    for (i = 0; i < K_STATE; i++)
    {
        ElementType probability_i = (qpreNode == 0 ? log(pi[i]) : log(A[snvOutPutPath[qpreNode - 1]][i]))+ log(B[i][T_obs]); // Initialize T1, use log to avoid data underflow
        for(int j = 0; j < N; ++j)
        {
            int changestate;
            generate_state_heap(probability_i, i, &(heap_total[j]), &changestate, -1);
        }
    }
    for(int i = 0; i < N; ++i)
    {
        element *heap_tem = heap_pre[i];
        heap_pre[i] = heap_total[i];
        heap_total[i] = heap_tem;
        initial_heap_element(heap_total[i]);
    }

    for (int j = qpreNode + 1; j < qsucNode + 1; j++)
    {
        T_obs = Obroute[j];           // Update current observation sequence value
        for (i = 0; i < K_STATE; i++) // Row
        {
            for (int k = 0; k < BeamSearchWidth; k++)
            {
                int preState = heap_pre[1][k + 1].State;
                ElementType preValue = heap_pre[1][k + 1].Value;
                scorearr[k] = preValue + log(A[preState][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid underflow, T1 already log
            }
            int scoreNo = 0;
            ElementType scoreMax = 0;
            int *p = &scoreNo;
            ElementType *q = &scoreMax;
            max_score(scorearr, BeamSearchWidth, p, q);
            ElementType probability_i = scoreMax; // Underflow
            for(int p = 1; p < N; ++p)
            {
                int changestate;
                generate_state_heap(probability_i, i, &(heap_total[p]), &changestate,
                    j > midpoint[p]+1 ? heap_pre[p][scoreNo + 1].T3_State : heap_pre[p][scoreNo + 1].State);
            }
        }
        for(int i = 0; i < N; ++i)
        {
            element *heap_tem = heap_pre[i];
            heap_pre[i] = heap_total[i];
            heap_total[i] = heap_tem;
            initial_heap_element(heap_total[i]);
        }
    }
    // Now stores intermediate node path and final node probability
    // Find last node path, and backtrack midpoint based on this, output
    if (qpreNode == 0 && qsucNode == ObserRouteLEN-1)
    {
        ElementType last_max = heap_pre[1][1].Value; // Backtrack point itself is negative, don't compare with 0
        int last_max_no = 0;
        // int last_max_state = heap_pre[1].State;
        for (int i = BeamSearchWidth / 2+1; i < BeamSearchWidth; i++)
        {
            if (last_max < heap_pre[1][i + 1].Value)
            {
                last_max_no=i;
                last_max = heap_pre[1][i + 1].Value;
            }
        }
        ElementType value_test=heap_pre[1][last_max_no+1].Value;
        snvOutPutPath[qsucNode] = heap_pre[1][last_max_no+1].State; // Second last node value //?Store max value
        for(int p = 1; p < N; ++p)
        {
            snvOutPutPath[midpoint[p]] = heap_pre[p][last_max_no+1].T3_State;
        }
        
        printf("Total memory usage (excluding output): %zu bytes\n", (sizeof(ElementType)+2*sizeof(int))*(BeamSearchWidth + 1)*N*2+sizeof(midpoint)+(sizeof(scorearr)+sizeof(last_max)+sizeof(last_max_no)+sizeof(T_obs))*MAX_THREADS);
        printf("Total memory usage: %zu bytes\n", (sizeof(ElementType)+2*sizeof(int))*(BeamSearchWidth + 1)*N*2+sizeof(midpoint)+sizeof(snvOutPutPath)+(sizeof(scorearr)+sizeof(last_max)+sizeof(last_max_no)+sizeof(T_obs))*MAX_THREADS);
    }
    else // All others execute
    {
        int connt = snvOutPutPath[qsucNode]; // Find current end point state value
        for(int p = 1; p < N; ++p)
        {
            snvOutPutPath[midpoint[p]] = Find_T3_State(&(heap_pre[p]), connt);
        }
    }

    for(int i = 0; i < N; ++i)
    {
        free(heap_pre_matrix[i]);
        free(heap_total_matrix[i]);
    }
    free(heap_pre_matrix);
    free(heap_total_matrix);

    EnQueue(Q,qpreNode,midpoint[1]);
    for(int p = 1; p < N; ++p)
    {
        EnQueue(Q,midpoint[p]+1,midpoint[p+1]);
    }
}

void Viterbifirst(int qpreNode, int qsucNode) // Case when >3
{

    int TobsLenth = ObserRouteLEN;
    if (qsucNode - qpreNode < 0) // If successor-predecessor <0, end
        return;
    // (TobsLenth <= 1)
    //  return;
    int i;
    ElementType scorearr[BeamSearchWidth] = {0}; // Store all probability values for finding max probability
    int T_obs;                                // Initial observation sequence value, first column observation value   *Current sequence start observation value
    int midpoint = (qpreNode + qsucNode) / 2; // path intermediate node, after execution gets current Path[midpoint]
    // snvOutPutPath[midpoint - 1] = -1; // init path[midpoint-1]
    element heap_1[BeamSearchWidth + 1];
    element heap_2[BeamSearchWidth + 1];
    element *heap_pre = heap_1;
    element *heap_total = heap_2;
    for (int j = qpreNode; j < qsucNode + 1; j++)
    {
        if (j == qpreNode)
        {
            if (qpreNode == 0) // Only initialize T table for first column, when head node is 0
            {
                T_obs = Obroute[qpreNode];

                initial_heap_element(heap_total);
                initial_heap_element(heap_pre);

                for (i = 0; i < K_STATE; i++)
                {
                    double B_data=log(B[i][T_obs]);
                    double pi_data=log(pi[i]);
                    ElementType probability_i = log(pi[i]) + log(B[i][T_obs]); // Initialize T1, use log to avoid data underflow
                    // if (i<BeamSearchWidth-1){
                    //     insert_binary_tree(position,probability_i ,i);
                    //     num->Value++;
                    //     printf("total number is %d ,new element is %lf  ,state is %d  \n",num->Value,position->Value,position->State);
                    //     position++;
                    //     }
                    // else if (i==BeamSearchWidth-1){
                    //     insert_binary_tree(position,probability_i ,i);
                    //     num->Value++;
                    //     printf("total number is %d  ,new element is %lf  ,state is %d  \n",num->Value,position->Value,position->State);
                    //     create_max_heap(heap_total);
                    //     //Output sorted result
                    //     print_heap_element(heap_total);
                    //     }
                    // else{
                    //     if (probability_i>heap_total[1].Value){
                    //         printf("\nChange:new element value is %lf state is %d  ,Total min element is %lf  state is %d  \n",probability_i,i,heap_total[1].Value,heap_total[1].State);
                    //         heap_total[1].Value=probability_i;
                    //         heap_total[1].State=i;
                    //         create_max_heap(heap_total);
                    //         print_heap_element(heap_total);
                    //         }
                    //         // Hidden
                    //     else{
                    //         printf("Keep:new element value is %lf  state is %d  ,Total min element is %lf  state is %d  \n",probability_i,i,heap_total[1].Value,heap_total[1].State);
                    //         }
                    //     }
                    int changestate;
                    int changgelabel = generate_state_heap(probability_i, i, &heap_total, &changestate, -1);
                    // chaneg_T2_i(changestate,changgelabel,i,-1);
                    // snvT3[i] = -1;                            // Initialize T3
                }

                element *heap_tem = heap_pre;
                heap_pre = heap_total;
                heap_total = heap_tem;
                initial_heap_element(heap_total);


                // Output sorted result, for testing
                //  print_heap_element(&heap_pre);
                //  snvT1[i] = log(pi[i]) + log(B[i][T_obs]); // Initialize T1, use log to avoid data underflow
                // printf("over:\n");
            }
            else // Other cases initialize column, if not initial column, means previous mid column, so directly find corresponding state, others initialize to 0
            {
                initial_heap_element(heap_total);
                initial_heap_element(heap_pre);

                T_obs = Obroute[qpreNode];                    // Update current observation sequence value
                int State_temp = snvOutPutPath[qpreNode - 1]; //  Current start corresponding state value
                for (i = 0; i < K_STATE; i++)                 // Row, for known start element start point (i), directly use i corresponding probability for subsequent state assignment
                {
                    // for (int k = 0; k < K_STATE; k++)
                    // {
                    //     scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid underflow, T1 already log
                    // }
                    // int scoreNo = 0;
                    // ElementType scoreMax = 0;
                    // int* p = &scoreNo;
                    // ElementType* q = &scoreMax;
                    // max_score(scorearr, K_STATE, p, q); //T2 stores index, T1 is probability
                    // snvTtmp[i] = scoreMax; // Underflow
                    // snvT2[i] = scoreNo;//First half doesn't need path storage
                    ElementType probability_i = log(A[State_temp][i]) + log(B[i][T_obs]);
                    int changestate;
                    int changgelabel = generate_state_heap(probability_i, i, &heap_total, &changestate, -1);
                    // chaneg_T2_i(changestate,changgelabel,i,State_temp);

                    // if (!changgelabel){
                    //     snvT2[i] =  State_temp;}
                    // else if(changgelabel==1)
                    // {
                    //     snvT2[i] =  State_temp;
                    //     snvT2[changestate] =  -1;
                    //     print("change stae %d value to %d ,input state %d value to %d\n",changestate,snvT2[changestate],i,snvT2[i]);
                    // }
                    // else{
                    //     snvT2[i] =  -1;
                    //     print("input stae %d value to %d\n",i,snvT2[i]);
                    // }
                    // snvT3[i] = -1;                            // Initialize T3
                }
                element *heap_tem = heap_pre;
                heap_pre = heap_total;
                heap_total = heap_tem;
                initial_heap_element(heap_total);


                // Output sorted result, for testing
                //  print_heap_element(&heap_pre);
                //  printf("over:\n");
            }
        }
        // Column pre+1 to midpoint column              0-4
        else if (j < midpoint + 1)
        {
            T_obs = Obroute[j];           // Update current observation sequence value
            for (i = 0; i < K_STATE; i++) // Row
            {
                for (int k = 0; k < BeamSearchWidth; k++)
                {
                    int preState = heap_pre[k + 1].State;
                    ElementType preValue = heap_pre[k + 1].Value;
                    ElementType A_value = log(A[preState][i]);
                    ElementType B_value = log(B[i][T_obs]);
                    scorearr[k] = preValue + log(A[preState][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid underflow, T1 already log
                }
                int scoreNo = 0;
                ElementType scoreMax = 0;
                int *p = &scoreNo;
                ElementType *q = &scoreMax;
                max_score(scorearr, BeamSearchWidth, p, q);
                ElementType probability_i = scoreMax; // Underflow
                int T3_state_test=heap_pre[scoreNo + 1].State;
                int changestate;
                int changgelabel = generate_state_heap(probability_i, i, &heap_total, &changestate, heap_pre[scoreNo + 1].State);
                // chaneg_T2_i(changestate,changgelabel,i,-1);//First half doesn't need path storage
                // snvT2[i] = scoreNo;//First half doesn't need path storage
            }
            // printf("over:\n");
            element *heap_tem = heap_pre;
            heap_pre = heap_total;
            heap_total = heap_tem;
            initial_heap_element(heap_total);
            // Output sorted result, for testing
            //  print_heap_element(&heap_pre);
        }
        // Column midpoint+1                               5
        else if (j == midpoint + 1)
        {
            T_obs = Obroute[midpoint + 1];
            for (i = 0; i < K_STATE; i++) // Row
            {
                for (int k = 0; k < BeamSearchWidth; k++)
                {
                    int preState = heap_pre[k + 1].State; // Row 0 stores info, so k+1
                    int pre_T3_State = heap_pre[k + 1].T3_State; // Row 0 stores info, so k+1
                    ElementType A_value = log(A[preState][i]);
                    ElementType B_value = log(B[i][T_obs]);
                    ElementType preValue = heap_pre[k + 1].Value;
                    scorearr[k] = preValue + log(A[preState][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid underflow, T1 already log
                }
                int scoreNo = 0;
                ElementType scoreMax = 0;
                int *p = &scoreNo;
                ElementType *q = &scoreMax;
                max_score(scorearr, BeamSearchWidth, p, q);
                ElementType probability_i = scoreMax; // Underflow
                int changestate;
                int changgelabel = generate_state_heap(probability_i, i, &heap_total, &changestate, heap_pre[scoreNo + 1].State);
                // chaneg_T2_i(changestate,changgelabel,i,heap_pre[scoreNo+1].State);//Second half path starts saving T2; merged with heap operation
            }
            // printf("over:\n");
            element *heap_tem = heap_pre;
            heap_pre = heap_total;
            heap_total = heap_tem;
            initial_heap_element(heap_total);


            // Output sorted result, for testing
            //  print_heap_element(&heap_pre);
            //  Change_T3_element(&heap_pre,snvT3);

            // for (i = 0; i < K_STATE; i++){ // Copy temp probability values, and update T3 column, T2 and T3 iterative update
            //         snvT3[i] = snvT2[i];
            // }
        }
        // Column midpoint+2 to qsucNode column         6-8
        else 
        {
            // if(j=qsucNode)               //test
            //     printf("over:\n");
            T_obs = Obroute[j];           // Update current observation sequence value
            for (i = 0; i < K_STATE; i++) // Row
            {
                for (int k = 0; k < BeamSearchWidth; k++)
                {
                    int preState = heap_pre[k + 1].State; // Row 0 stores info, so k+1
                    ElementType preValue = heap_pre[k + 1].Value;
                    ElementType A_value = log(A[preState][i]);
                    ElementType B_value = log(B[i][T_obs]);
                    scorearr[k] = preValue + log(A[preState][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid underflow, T1 already log
                }
                int scoreNo = 0;
                ElementType scoreMax = 0;
                int *p = &scoreNo;
                ElementType *q = &scoreMax;
                max_score(scorearr, BeamSearchWidth, p, q);
                ElementType probability_i = scoreMax; // Underflow
                int changestate;
                int T3_state =  heap_pre[scoreNo + 1].T3_State;            // Find_T3_State(&heap_pre, heap_pre[scoreNo + 1].State);
                int changgelabel = generate_state_heap(probability_i, i, &heap_total, &changestate, T3_state); // Merged with heap operation
                // chaneg_T2_i(changestate,changgelabel,i,snvT3[heap_pre[scoreNo+1].State]);//Second half path starts saving T2
            } // Assign max value index to

            element *heap_tem = heap_pre;
            heap_pre = heap_total;
            heap_total = heap_tem;
            initial_heap_element(heap_total);
            // Output sorted result, for testing
            //  print_heap_element(&heap_pre);
            //  Change_T3_element(&heap_pre,snvT3);

            // for (i = 0; i < K_STATE; i++) // Copy temp probability values and update T3 column, T2 and T3 iterative update
            // {
            //     // snvT1[i] = snvTtmp[i];
            //     snvT3[i] = snvT2[i];
            // }
        }
    }
    // Now stores intermediate node path and final node probability
    // Find last node path, and based on this backtrack midpoint, output
    int mi = (ObserRouteLEN - 1) / 2;
    if (midpoint == mi) // Only executed first time
    {
        ElementType last_max = heap_pre[1].Value; // Backtrack point itself is negative, don't compare with 0
        int last_max_no = 0;
        // int last_max_state = heap_pre[1].State;
        for (int i = BeamSearchWidth / 2+1; i < BeamSearchWidth; i++)
        {
            if (last_max < heap_pre[i + 1].Value)
            {
                last_max_no=i;
                last_max = heap_pre[i + 1].Value;
                // last_max_state = heap_pre[i + 1].State;
            }
        }
        ElementType value_test=heap_pre[last_max_no+1].Value;
        int connttest = heap_pre[last_max_no+1].State; // Find current end point state value
        int T3_Statetest = heap_pre[last_max_no+1].T3_State; // Find current end point T3 state value

        snvOutPutPath[qsucNode] = heap_pre[last_max_no+1].State; // Second last node value //?Store max value

        snvOutPutPath[midpoint] = heap_pre[last_max_no+1].T3_State;    //Find_T3_State(&heap_pre, last_max_no);
        // snvOutPutPath[midpoint] = snvT3[last_max_no]; // midpoint node value, final T2 same as T3

        // printf("heap_total memory usage: %zu bytes\n", sizeof(heap_1));
        // printf("heap_pre memory usage: %zu bytes\n", sizeof(heap_2));
        // printf("scorearr memory usage: %zu bytes\n", sizeof(scorearr));
        // printf("snvOutPutPath memory usage: %zu bytes\n", sizeof(snvOutPutPath));
        // printf("last_max memory usage: %zu bytes\n", sizeof(last_max));
        // printf("last_max_no memory usage: %zu bytes\n", sizeof(last_max_no));
        // printf("T_obs memory usage: %zu bytes\n", sizeof(T_obs));
        // printf("Total memory usage (excluding output): %zu bytes\n", sizeof(heap_1) + sizeof(heap_2) + sizeof(scorearr) + sizeof(last_max) + sizeof(last_max_no) + sizeof(T_obs));
        // printf("Total memory usage: %zu bytes\n", sizeof(heap_1) + sizeof(heap_2) + sizeof(scorearr) + sizeof(snvOutPutPath) + sizeof(last_max) + sizeof(last_max_no) + sizeof(T_obs));


        printf("\n log lastmax %lf", last_max);  // log value test
        printf("\n lastmax %lf", exp(last_max)); // Max path probability value
    }
    else // All others execute
    {
        // if(qpreNode==320&&qsucNode==383)
        // {
        //     printf("over:\n");
        // }
        int connt = snvOutPutPath[qsucNode]; // Find current end point state value
        snvOutPutPath[midpoint] = Find_T3_State(&heap_pre, connt);
        // snvOutPutPath[midpoint] = snvT3[connt]; // Output within current path range, output T3 value at this state point
    }

}

void printRoute() // Output path
{

    printf("\nTotal states: %d, pruned remaining states: %d, most likely hidden state path length: %d , path is:\n",K_STATE,BeamSearchWidth,ObserRouteLEN);

    for (int i = 0; i < ObserRouteLEN; i++)
    {
        printf(" %d,", snvOutPutPath[i]);
    }
    printf("\n");
}
int maxQueueSize = 1; // Record max queue length
// Worker thread function
DWORD WINAPI Worker(LPVOID arg) {
    ThreadPool* pool = (ThreadPool*)arg;
    while (1) {
        int pre = 0,suc = ObserRouteLEN-1, midpoint;
        int *qpre = &pre,*qsuc = &suc;
        EnterCriticalSection(&pool->lock); // Enter critical section
        int currentQueueSize;
        while((currentQueueSize = ((pool->Q).rear - (pool->Q).front + ObserRouteLEN) % ObserRouteLEN) == 0 
                && pool->task_count < TotalTask)
        {
            SleepConditionVariableCS(&pool->pool_wake, &pool->lock, INFINITE);
        }
        if (currentQueueSize > maxQueueSize)
        {
            maxQueueSize = currentQueueSize;
        }
        if (DeQueue(&(pool->Q),qpre,qsuc) == 0) {       // Check if task queue is empty
            LeaveCriticalSection(&pool->lock);
            break;
        }
        pool->task_count++;
        LeaveCriticalSection(&pool->lock);                  // Leave critical section
        midpoint = (pre + suc) / 2;

        if (suc - pre > 2)
        {
            Viterbifirst(pre, suc);
            EnterCriticalSection(&pool->lock); // Enter critical section
            EnQueue(&(pool->Q), pre, midpoint);
            EnQueue(&(pool->Q), midpoint + 1, suc);
            LeaveCriticalSection(&pool->lock); // Leave critical section
        }
        else if (suc == pre + 2)        
        {
            Viterbifirst(pre, suc);
            EnterCriticalSection(&pool->lock); // Enter critical section
            EnQueue(&(pool->Q), pre, midpoint);
            LeaveCriticalSection(&pool->lock); // Leave critical section
        }           
        else //pre=suc+1,pre = suc
        {
            Viterbifirst(pre, suc);
        }
        WakeAllConditionVariable(&pool->pool_wake);
    }
    return 0;
}
// Initialize thread pool
void ThreadPoolInit(ThreadPool* pool) {
    InitQueue(&(pool->Q)); 
    pool->task_count = 0;
    pool->active_threads = MAX_THREADS;
    InitializeCriticalSection(&pool->lock); // Initialize critical section object
    InitializeConditionVariable(&pool->pool_wake);
    for (int i = 0; i < MAX_THREADS; ++i) {
        pool->threads[i] = CreateThread(NULL, 0, Worker, (LPVOID)pool, 0, NULL); // Create worker thread
    }
}
// Destroy thread pool
void DestroyThreadPool(ThreadPool* pool) {
    for (int i = 0; i < MAX_THREADS; ++i) {
        WaitForSingleObject(pool->threads[i], INFINITE); // Wait for worker thread to finish
        CloseHandle(pool->threads[i]); // Close thread handle
    }
    DeleteCriticalSection(&pool->lock); // Delete critical section object
}
void tracebackroute() // space main body         Call main code just created
{
    int TobsLenth = ObserRouteLEN; // Observation path length
    int pre = 0;             // Current processing path start
    int suc = TobsLenth - 1; // End point
    int *qpre = &pre;        // Declare pointers qpre and qsuc pointing to pre and suc, used to update start and end points in function.
    int *qsuc = &suc;
    ThreadPool pool;
    ThreadPoolInit(&pool);
    // printf("Queue Q memory usage: %zu bytes\n", sizeof(Q));
    int Ndivide = MAX_THREADS;
    if(suc - pre >= 2*Ndivide)
    {
        TotalTask = ObserRouteLEN-Ndivide;
        ViterbiNDivide(pre,suc,Ndivide,&(pool.Q));
        WakeAllConditionVariable(&pool.pool_wake);
    }
    else
    {
        TotalTask = ObserRouteLEN-1;
        EnQueue(&(pool.Q), pre, suc);
        WakeConditionVariable(&pool.pool_wake);
    }

    DestroyThreadPool(&pool);
    ShowQueue(&(pool.Q));
    printf("Queue Q max data size: %d\n", maxQueueSize);
    printf("task:%d\n",pool.task_count);
}

int exmpleSNVviter()
{

    int excu_i = 0;
    char* str_MatrixA="E:\\Code\\Ccode\\test\\Nvviter\\new512data\\A_K3965_T256_prob0.075.txt";
    char* str_MatrixB="E:\\Code\\Ccode\\test\\Nvviter\\new512data\\B_K3965_T256_prob0.075.txt";
    char* str_MatrixPI="E:\\Code\\Ccode\\test\\Nvviter\\new512data\\Pi_K3965_T256_prob0.075.txt";
    char* str_ObRoute="E:\\Code\\Ccode\\test\\Nvviter\\new512data\\ob_K3965_T256_prob0.075.txt";
    InitOutputpath();
    InitMatrixA(str_MatrixA);
    InitMatrixB(str_MatrixB);
    InitMatrixPI(str_MatrixPI);
    InitObRoute(str_ObRoute);
    QueryPerformanceFrequency(&tc); // time calculate execution time
    QueryPerformanceCounter(&t1);   // time
    tracebackroute();
    QueryPerformanceCounter(&t2);                                                                 // time
    printf("\nNvvVITERBI_time:%lf s", (double)(t2.QuadPart - t1.QuadPart) / (double)tc.QuadPart); // time
    printRoute();                                                                                 // Output path
    printf("\nthank\n");
    return 1;
}

int main()
{

    if (!exmpleSNVviter()) // exmplesnv

    {
        printf("error");
        return 0;
    }

    printf("success");
    return 1;
}