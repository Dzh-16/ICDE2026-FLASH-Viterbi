#define _CRT_SECURE_NO_WARNINGS // FOPEN

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <windows.h>


#define K_STATE 512 // State space size                       Modified to same value as K
#define T_STATE 50 // Observation state size

#define ObserRouteLEN 512 //Observation path length              Modified to same value as T

#define MAX_THREADS 7
clock_t start_time;
clock_t end_time;//COUNT TIME
LARGE_INTEGER t1, t2, tc;

typedef float ElementType;    //MAX HEAP sorting element type is float, i.e., transition probability unit is double

ElementType A[K_STATE][K_STATE];//When array dimension is too large, cannot allocate too much memory, use malloc
ElementType B[K_STATE][T_STATE];
ElementType pi[K_STATE];
int Obroute[ObserRouteLEN];

int snvOutPutPath[ObserRouteLEN]; // output

typedef struct Queue
{
    int* preNode; //Previous node of the path
    int* sucNode;//Next node of the path
    int  front; //Head pointer
    int  rear; //Tail pointer
}Queue;

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

void InitOutputpath()
{
    int i = 0;
    for (i = 0; i < ObserRouteLEN+1; i++)
    {
        snvOutPutPath[i] = -1;
    }
}

void InitQueue(Queue* Q)        //Initialize queue
{
    //Allocate storage space for the queue, for predecessor and successor queues
    Q->preNode = (int*)malloc(sizeof(int) * ObserRouteLEN);//Allocate integer storage space of length observation path length for pointer variable
    Q->sucNode = (int*)malloc(sizeof(int) * ObserRouteLEN);
    if ((Q->preNode != NULL) && (Q->sucNode != NULL))
        Q->front = Q->rear = 0;
    //Initially the queue is empty, both head and tail pointers point to position 0
    else
    {
        printf("allocation is failed !!");
        return;
    }
}

void EnQueue(Queue* Q, int preside, int sucside)        //Enqueue
{
    //Check if circular queue is full
    if (((Q->rear + 1) % ObserRouteLEN) == Q->front)   //Full condition is next position of tail pointer (via modulo operation (Q->rear + 1) % ObserRouteLEN) equals head pointer position (Q->front)
        return;
    //Queue not full, enqueue data
    Q->preNode[Q->rear] = preside;//Predecessor boundary enqueued
    Q->sucNode[Q->rear] = sucside;//Successor boundary enqueued
    //Update tail pointer position
    Q->rear = (Q->rear + 1) % ObserRouteLEN;
}

int DeQueue(Queue* Q, int* preside, int* sucside)       //Dequeue
{
    //Check if circular queue is empty
    if (Q->front == Q->rear)
        return 0;
    //Before dequeuing, save predecessor and successor node values
    *preside = Q->preNode[Q->front];
    *sucside = Q->sucNode[Q->front];
    //If not empty, implement circular dequeue
    Q->front = (Q->front + 1) % ObserRouteLEN;
    return 1;
}

void ShowQueue(Queue* Q)                
{
    //Traverse elements in circular queue and print data
    for (int i = Q->front; i != Q->rear; i = (i + 1) % ObserRouteLEN)
    {
        printf("(%d->%d), ", Q->preNode[i], Q->sucNode[i]);
        //This operation implements circular traversal
    }
    printf("\n");
}

int Length(Queue* Q)            
{
    //Calculate difference between tail and head pointer positions
    int len = Q->rear - Q->front;
    //If positive, len is queue length; if negative, ObserRouteLEN+len is queue length
    len = (len > 0) ? len : ObserRouteLEN + len;
    return len;
}

void max_score(ElementType arr[], int arr_count, int* max_temp_no, ElementType* max_temp)       //Find maximum probability value and index in queue
{
    if (NULL == arr)
        return;
    int max_no = 0;                 //Probability value No.
    ElementType max = arr[0];       //Probability value
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
        FILE *fpRead = fopen(str_MatrixA, "rb+"); // After program successfully opens file, fopen() returns file pointer. File pointer type is FILE

        if (fpRead == NULL)
        {
            printf("open fail errno = %d reason = %s \n", errno, strerror(errno));
            return;
        }
        int retnum = 0;
        for (i = 0; i < K_STATE * K_STATE; i++)
        {
            retnum = fscanf(fpRead, "%f", &tmpt[i]); // Use fscanf to read a double type data from file and store in tmpt[i]
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

void InitMatrixB(char* str_MatrixB)
{
    int i = 0, j = 0;
    ElementType *tmpt = (ElementType *)malloc(sizeof(ElementType) * K_STATE * T_STATE);
    if (tmpt)
    {
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
}

void InitObRoute(char* str_ObRoute)
{
    int i, j;
    int tmpt[ObserRouteLEN] = { 0 };
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
}

void ViterbiNDivide(int qpreNode, int qsucNode, int N, Queue *Q)
{
    int TobsLenth = ObserRouteLEN;
    if (qsucNode - qpreNode < 2*N)  //Less than 6 cannot divide by 4
        return;
    int i, j;
    ElementType scorearr[K_STATE] = { 0 }; // Store all probability values for finding maximum probability

    int T_obs = Obroute[qpreNode]; // Initial observation sequence value, first column observation value *Current sequence start observation value
    int midpoint[N+1], gap_length = (qsucNode - qpreNode) / N, gap_extra = (qsucNode - qpreNode) % N;
    midpoint[0] = qpreNode, midpoint[N] = qsucNode; //0 has no meaning
    for(int i = 1; i < N; ++i)
    {
        midpoint[i] = midpoint[i-1]+gap_length;
        if(gap_extra > 0) gap_extra--, midpoint[i]++;
    }
    ElementType snvT1[K_STATE];
    ElementType snvTtmp[K_STATE];
    int** snv = (int**)malloc(sizeof(int*)*K_STATE);
    int** snv_tmp = (int**)malloc(sizeof(int*)*K_STATE);
    for (i = 0; i < K_STATE; i++)
    {
        snv[i] = (int*)malloc(sizeof(int)*N);
        snv_tmp[i] = (int*)malloc(sizeof(int)*N);
        snvTtmp[i] = (qpreNode == 0 ? log(pi[i]) : log(A[snvOutPutPath[qpreNode-1]][i])) + log(B[i][T_obs]);
        snvT1[i] = snvTtmp[i];
        for(int j = 1; j < N; ++j)
        {
            snv_tmp[i][j] = (qpreNode == 0 ? -1 : snvOutPutPath[qpreNode-1]);
            snv[i][j] = snv_tmp[i][j];
        }
    }
    for (j = qpreNode+1; j < qsucNode+1; j++)      
    {
        T_obs = Obroute[j];           // Update current observation sequence value
        for (i = 0; i < K_STATE; i++)
        {
            for (int k = 0; k < K_STATE; k++)
            {
                scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
            }
            int scoreNo = 0;
            ElementType scoreMax = 0;
            int* p = &scoreNo;
            ElementType* q = &scoreMax;
            max_score(scorearr, K_STATE, p, q);
            snvTtmp[i] = scoreMax;

            for(int p = 1; p < N; ++p)
            {
                if(j > midpoint[p]+1) snv_tmp[i][p] = snv[scoreNo][p];
                else snv_tmp[i][p] = snv[i][p] = scoreNo;
            }
        }
        for (i = 0; i < K_STATE; i++) 
        {
            snvT1[i] = snvTtmp[i];
            for(int p = 1; p < N; ++p)
            {
                if(j > midpoint[p]+1) snv[i][p] = snv_tmp[i][p];
            }
        }
    }
    
    //Now stores intermediate node path and last node's probability
    //Find last node's path, and based on this backtrack path midpoint, output
    if (qpreNode == 0 && qsucNode == ObserRouteLEN-1) //Only executed first time
    {
        ElementType last_max = snvT1[0];//Backtrack point itself is negative, don't compare with 0
        int last_max_no = 0;
        for (int i = 0; i < K_STATE; i++)
        {
            if (last_max < snvT1[i])
            {
                last_max = snvT1[i];
                last_max_no = i;
            }
        }

        snvOutPutPath[qsucNode] = last_max_no;        // Second last node
        for(int p = 1; p < N; ++p)
        {
            snvOutPutPath[midpoint[p]] = snv[last_max_no][p];
        }
        printf("Total memory occupied (excluding output): %zu bytes\n", (sizeof(int)*K_STATE*2+sizeof(snvT1)+sizeof(snvTtmp))*N+(sizeof(scorearr)+sizeof(last_max)+sizeof(last_max_no)+sizeof(T_obs))*MAX_THREADS+sizeof(midpoint));
        printf("Total memory occupied: %zu bytes\n", (sizeof(int)*K_STATE*2+sizeof(snvT1)+sizeof(snvTtmp))*N+sizeof(snvOutPutPath)+(sizeof(scorearr)+sizeof(last_max)+sizeof(last_max_no)+sizeof(T_obs))*MAX_THREADS+sizeof(midpoint));
        printf("\n log lastmax%f", last_max);//log value test
        printf("\n lastmax%f", exp(last_max));//Maximum path probability
    }
    else//All others execute
    {
        int connt = snvOutPutPath[qsucNode];//Find current end point's state value
        for(int p = 1; p < N; ++p)
        {
            snvOutPutPath[midpoint[p]] = snv[connt][p];
        }
    }

    for(int i = 0; i < K_STATE; ++i)
    {
        free(snv[i]);
        free(snv_tmp[i]);
    }
    free(snv);
    free(snv_tmp);

    EnQueue(Q,qpreNode,midpoint[1]);
    for(int p = 1; p < N; ++p)
    {
        EnQueue(Q,midpoint[p]+1,midpoint[p+1]);
    }
}

void Viterbifirst(int qpreNode, int qsucNode)       //Case when greater than 3
{
    int TobsLenth = ObserRouteLEN;
    if (qsucNode - qpreNode < 0)        //Successor - predecessor less than 0, end
        return;
    double test_pi,test_B,or_B,test_A;
    int i, j;
    ElementType scorearr[K_STATE] = { 0 }; // Store all probability values for finding maximum probability

    int T_obs = Obroute[qpreNode]; // Initial observation sequence value, first column observation value *Current sequence start observation value
    int midpoint = (qpreNode + qsucNode) / 2;  //path's intermediate node, after running get current Path[midpoint]
    ElementType snvT1[K_STATE];       // T1_table,prob
    ElementType snvTtmp[K_STATE];       // T1_table,save for last prob
    int snvT2[K_STATE];               // T2_table,path
    int snvT3[K_STATE];               // Mid Table,midpath
    if (qpreNode == 0) //Only initialize T table for first column, when head node is 0
    {
        for (i = 0; i < K_STATE; i++)
        {
            test_pi = log(pi[i]);
            or_B   = B[i][T_obs];
            test_B = log(B[i][T_obs]);
            snvT1[i] = log(pi[i]) + log(B[i][T_obs]); // Initialize T1, use log to avoid data underflow
            snvT2[i] = -1;                            // Initialize T2
            snvT3[i] = -1;                            // Initialize T3
        }
    }
    else//Other cases initialize the same, if not initial column, means this is previous mid column, so directly find corresponding state, others initialize to 0
    {
        T_obs = Obroute[qpreNode];           // Update current observation sequence value
        int State_temp=snvOutPutPath[qpreNode-1]; //  Current start corresponding state value
        for (i = 0; i < K_STATE; i++)
        {
            test_A = log(A[State_temp][i]);
            test_B = log(B[i][T_obs]);
            snvT2[i] =  State_temp;
            snvTtmp[i]= log(A[State_temp][i]) + log(B[i][T_obs]);
        }
        for (i = 0; i < K_STATE; i++) // Copy temporary probability values
        {
            snvT1[i] = snvTtmp[i];
        }
    }
    // Column pre+1 to midpoint
    for (j = qpreNode+1; j < midpoint+1; j++)      
    {
        T_obs = Obroute[j];           // Update current observation sequence value
        for (i = 0; i < K_STATE; i++)
        {
            for (int k = 0; k < K_STATE; k++)
            {
                double test_A,test_B;
                test_A = log(A[k][i]);
                test_B = log(B[i][T_obs]);
                scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
            }
            int scoreNo = 0;
            ElementType scoreMax = 0;
            int* p = &scoreNo;
            ElementType* q = &scoreMax;
            max_score(scorearr, K_STATE, p, q);
            snvTtmp[i] = scoreMax; // Underflow
            snvT2[i] = scoreNo;//First half doesn't need to store path
        }
        int lastno=snvT2[K_STATE-1];
        ElementType lastmax=snvTtmp[K_STATE-1];
        for (i = 0; i < K_STATE; i++) // Copy temporary probability values
        {
            snvT1[i] = snvTtmp[i];
        }
    }

    // Column midpoint+1

    T_obs = Obroute[midpoint+1];           // Update current observation sequence value,T2[midpoint+1]
    for (i = 0; i < K_STATE; i++)
    {
        for (int k = 0; k < K_STATE; k++)
        {
            double test_A,test_B,test_T1;
            test_A = log(A[k][i]);
            test_B = log(B[i][T_obs]);
            test_T1 = snvT1[k];
            scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
        }
        int scoreNo = 0;
        ElementType scoreMax = 0;
        int* p = &scoreNo;
        ElementType* q = &scoreMax;
        max_score(scorearr, K_STATE, p, q);
        snvTtmp[i] = scoreMax; // Underflow
        snvT2[i] = scoreNo;//Second half path starts saving T2
        snvT3[i] = scoreNo;//Predecessor path of midpoint node
    }
    for (i = 0; i < K_STATE; i++) // Copy temporary probability values
    {
        snvT1[i] = snvTtmp[i];
    }

    // Column midpoint+2 to qsucNode
    for (j = midpoint + 2; j < qsucNode + 1; j++)      
    {
        T_obs = Obroute[j];           // Update current observation sequence value
        for (i = 0; i < K_STATE; i++)
        {
            for (int k = 0; k < K_STATE; k++)
            {
                double test_A,test_B,test_T1;
                test_A = log(A[k][i]);
                test_B = log(B[i][T_obs]);
                test_T1 = snvT1[k];
                scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
            }
            int scoreNo = 0;
            ElementType scoreMax = 0;
            int* p = &scoreNo;
            ElementType* q = &scoreMax;
            max_score(scorearr, K_STATE, p, q);
            snvTtmp[i] = scoreMax; // Underflow
            snvT2[i] = snvT3[scoreNo];//Take value in T3 corresponding to predecessor node index node as current column to mid column path
        }
        for (i = 0; i < K_STATE; i++) // Copy temporary probability values, and update T3 column, T2 and T3 iterative update
        {
            snvT1[i] = snvTtmp[i];
            snvT3[i] = snvT2[i];
        }
    }
    
    //Now stores intermediate node path and last node's probability
    //Find last node's path, and based on this backtrack path midpoint, output
    int mi = (ObserRouteLEN - 1) / 2;
    if (midpoint == mi ) //Only executed first time
    {
        ElementType last_max = snvT1[0];//Backtrack point itself is negative, don't compare with 0
        int last_max_no = 0;
        for (int i = 0; i < K_STATE; i++)
        {
            if (last_max < snvT1[i])
            {
                last_max = snvT1[i];
                last_max_no = i;
            }
        }

        snvOutPutPath[qsucNode] = last_max_no;        // Second last node
        snvOutPutPath[midpoint] = snvT2[last_max_no]; // midpoint node value, finally T2 and T3 are same

        printf("\n log lastmax%f", last_max);//log value test
        printf("\n lastmax%f", exp(last_max));//Maximum path probability
    }
    else//All others execute
    {
        int connt = snvOutPutPath[qsucNode];//Find current end point's state value
        snvOutPutPath[midpoint] = snvT3[connt]; //Output within current path range, output value in T3 to this state point
    }
}

void viter1(int qpreNode, int qsucNode)//pre + 1 = suc,mid = pre  End interval case of size 2
{
    int TobsLenth = ObserRouteLEN;
    if (qsucNode - qpreNode <= 0)
        return;
    int i, j;
    ElementType scorearr[K_STATE] = { 0 }; // Store all probability values for finding maximum probability
    int T_obs = Obroute[qpreNode]; // Initial observation sequence value

    ElementType snvT1[K_STATE];       // T1_table,prob
    ElementType snvTtmp[K_STATE];       // T1_table,save for last prob
    int snvT2[K_STATE];               // T2_table,path
    int snvT3[K_STATE];               // Mid Table,midpath
    
    if (qpreNode == 0) //Only initialize T table for first column
    {
        for (i = 0; i < K_STATE; i++)
        {
            snvT1[i] = log(pi[i]) + log(B[i][T_obs]); // Initialize T1, use log to avoid data underflow
            snvT2[i] = -1;                            // Initialize T2
            snvT3[i] = -1;                            // Initialize T3
        }
    }
    else//Other cases initialize the same
    {
        T_obs = Obroute[qpreNode];           // Update current observation sequence value
        int State_temp=snvOutPutPath[qpreNode-1]; //  Current start corresponding state value
        for (i = 0; i < K_STATE; i++)
        {
            snvT2[i] =  State_temp;
            snvTtmp[i]= log(A[State_temp][i]) + log(B[i][T_obs]);
        }
        for (i = 0; i < K_STATE; i++) // Copy temporary probability values
        {
            snvT1[i] = snvTtmp[i];
        }
    }

    // Column midpoint

    T_obs = Obroute[qsucNode];           // Update current observation sequence value,T2[midpoint+1]
    for (i = 0; i < K_STATE; i++)
    {
        for (int k = 0; k < K_STATE; k++)
        {
            scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
        }
        int scoreNo = 0;
        ElementType scoreMax = 0;
        int* p = &scoreNo;
        ElementType* q = &scoreMax;
        max_score(scorearr, K_STATE, p, q);
        snvTtmp[i] = scoreMax; // Underflow
        snvT2[i] = scoreNo;//Second half path starts saving T2
    }
    for (i = 0; i < K_STATE; i++) // Copy temporary probability values
    {
        snvT1[i] = snvTtmp[i];
    }

    //Now stores intermediate node path and last node's probability
    //Find last node's path, and based on this backtrack path midpoint, output
    int connt = snvOutPutPath[qsucNode];
    snvOutPutPath[qpreNode] = snvT2[snvOutPutPath[qsucNode]]; //Output within current path range
}

void viter2(int qpreNode, int qsucNode)//pre+1 = mid,mid+1 = suc   End interval case of size 3
{
    int TobsLenth = ObserRouteLEN;
    if (qsucNode - qpreNode < 0)
        return;
    int i, j;
    ElementType scorearr[K_STATE] = { 0 }; // Store all probability values for finding maximum probability
    int T_obs = Obroute[qpreNode]; // Initial observation sequence value
    int midpoint = qpreNode +1;  //path's intermediate node, after running get current Path[midpoint]
    ElementType snvT1[K_STATE];       // T1_table,prob
    ElementType snvTtmp[K_STATE];       // T1_table,save for last prob
    int snvT2[K_STATE];               // T2_table,path
    int snvT3[K_STATE];               // Mid Table,midpath
    if (qpreNode == 0) //Only initialize T table for first column
    {
        for (i = 0; i < K_STATE; i++)
        {
            snvT1[i] = log(pi[i]) + log(B[i][T_obs]); // Initialize T1, use log to avoid data underflow
            snvT2[i] = -1;                            // Initialize T2
            snvT3[i] = -1;                            // Initialize T3
        }
    }
    else//Other cases initialize the same
    {
        T_obs = Obroute[qpreNode];           // Update current observation sequence value
        int State_temp=snvOutPutPath[qpreNode-1]; //  Current start corresponding state value
        for (i = 0; i < K_STATE; i++)
        {
            snvT2[i] =  State_temp;
            snvTtmp[i]= log(A[State_temp][i]) + log(B[i][T_obs]);
        }
        for (i = 0; i < K_STATE; i++) // Copy temporary probability values
        {
            snvT1[i] = snvTtmp[i];
        }
    }

    T_obs = Obroute[midpoint];           // Update current observation sequence value
    for (i = 0; i < K_STATE; i++)
    {
        for (int k = 0; k < K_STATE; k++)
        {
            scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
        }
        int scoreNo = 0;
        ElementType scoreMax = 0;
        int* p = &scoreNo;
        ElementType* q = &scoreMax;
        max_score(scorearr, K_STATE, p, q);
        snvTtmp[i] = scoreMax; // Underflow
    }
    for (i = 0; i < K_STATE; i++) // Copy temporary probability values
    {
        snvT1[i] = snvTtmp[i];
    }

    // Column midpoint

    T_obs = Obroute[midpoint + 1];           // Update current observation sequence value,T2[midpoint+1]
    for (i = 0; i < K_STATE; i++)
    {
        for (int k = 0; k < K_STATE; k++)
        {
            scorearr[k] = snvT1[k] + log(A[k][i]) + log(B[i][T_obs]); // Calculate all probability values, use log to avoid numerical underflow, where T1 is already log 
        }
        int scoreNo = 0;
        ElementType scoreMax = 0;
        int* p = &scoreNo;
        ElementType* q = &scoreMax;
        max_score(scorearr, K_STATE, p, q);
        snvTtmp[i] = scoreMax; // Underflow
        snvT2[i] = scoreNo;//Second half path starts saving T2
    }
    for (i = 0; i < K_STATE; i++) // Copy temporary probability values
    {
        snvT1[i] = snvTtmp[i];
    }

    //Now stores intermediate node path and last node's probability
    //Find last node's path, and based on this backtrack path midpoint, output
    snvOutPutPath[midpoint] = snvT2[snvOutPutPath[qsucNode]]; //Output within current path range
}

void printRoute()//Output path
{
    printf("\nMost probable hidden state path length is %d, path is:\n", ObserRouteLEN);

    for (int i = 0; i < ObserRouteLEN; i++)
    {
        printf(" %d,", snvOutPutPath[i]);
    }
    printf("\n");
}

int maxQueueSize = 1; // Record maximum queue length
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
            viter2(pre, suc);
            EnterCriticalSection(&pool->lock); // Enter critical section
            EnQueue(&(pool->Q), pre, midpoint);
            LeaveCriticalSection(&pool->lock); // Leave critical section
        }           
        else //pre=suc+1,pre = suc
        {
            viter1(pre, suc);
        }
        WakeAllConditionVariable(&pool->pool_wake);
    }
    return 0;
}

// Initialize thread pool
void ThreadPoolInit(ThreadPool* pool) {
    InitQueue(&(pool->Q));
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
        WaitForSingleObject(pool->threads[i], INFINITE); // Wait for worker thread to end
        CloseHandle(pool->threads[i]); // Close thread handle
    }
    DeleteCriticalSection(&pool->lock); // Delete critical section object
}

void tracebackroute()//space main body        Call code from main body
{
    int TobsLenth = ObserRouteLEN;      //Observation path length
    int pre = 0;                        //Current processing path start
    int suc = TobsLenth - 1;            //End
    int* qpre = &pre;                   //Declare pointers qpre and qsuc pointing to pre and suc respectively, used to update start and end points in function
    int* qsuc = &suc;
    ThreadPool pool;
    ThreadPoolInit(&pool);
    printf("Queue Q memory occupied: %zu bytes\n", sizeof(pool.Q));
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
    printf("Queue Q maximum data amount: %d\n", maxQueueSize);
}

int exmpleSNVviter()
{
    int excu_i = 0;
    char* str_MatrixA="A.txt";
    char* str_MatrixB="B.txt";
    char* str_MatrixPI="Pi.txt";
    char* str_ObRoute="ob.txt";
    InitOutputpath();
    InitMatrixA(str_MatrixA);
    InitMatrixB(str_MatrixB);
    InitMatrixPI(str_MatrixPI);
    InitObRoute(str_ObRoute);
    QueryPerformanceFrequency(&tc); // time calculate execution time
    QueryPerformanceCounter(&t1);   // time
    tracebackroute();
    QueryPerformanceCounter(&t2);                                                           // time
    printf("\nNvvVITERBI_time:%lf s", (double)(t2.QuadPart - t1.QuadPart) / (double)tc.QuadPart); // time
    printRoute();//Output path
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