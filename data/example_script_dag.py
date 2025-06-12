import numpy as np 
import random
import networkx as nx 
import sys,getopt


def create_B(n_observables = 100, n_states = 100, sd = 1): 
    
    ''' create matrix of uniform emission probabilities '''
    
    np.random.seed(sd) 

    B = np.random.uniform(0.1, 1, (n_states, n_observables))
    
    B = B/B.sum(axis=1)[:,None]
    
    return B 

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"-h-s:-n:-K:-T:-b:-p:")
    except getopt.GetoptError:
        print('example_script_dag.py -s <seed> -n <n_ob> -K <K> -T <T>')
        sys.exit(2)
    num = 0
    for opt,arg in opts:
        if opt == '-h':
            print('example_script_dag.py -s <seed> -n <n_ob> -K <K> -T <T>')
            sys.exit()
        elif opt in ("-s"):
            sd = int(arg)
            num += 1
        elif opt in ("-n"):
            n_ob = int(arg)
            num += 1
        elif opt in ("-K"):
            K = int(arg)
            num += 1
        elif opt in ("-T"):
            T = int(arg)
            num += 1
    if num != 4:
        print('example_script_dag.py -s <seed> -n <n_ob> -K <K> -T <T>')
        sys.exit(2)

    random.seed(sd)
    y = [random.randint(0,n_ob-1) for _ in range(T)] 
    n_states = K

    # generate random DAG graph - sample edges with probability one 
    G=nx.gnp_random_graph(n_states,0.9,directed=True)
    DAG = nx.DiGraph([(u,v,{'weight':random.uniform(0,1)}) for (u,v) in G.edges() if u<v])        
    A = nx.to_numpy_array(DAG)
    A = A/A.sum(axis=1)
    A = np.nan_to_num(A)
        
    # generate emission probabilities 
    B = create_B(n_observables = n_ob,n_states = K, sd = sd)  
        
    # uniform initial probabilities 
    pi = np.full(n_states, 1 / n_states)

    np.savetxt(f'A_K{K}_T{T}_DAG.txt', A, fmt='%.16f')
    np.savetxt(f'B_K{K}_T{T}_DAG.txt', B, fmt='%.16f')
    np.savetxt(f'Pi_K{K}_T{T}_DAG.txt', pi, fmt='%.16f', newline=' ')
    np.savetxt(f'ob_K{K}_T{T}_DAG.txt', y, fmt='%d', newline=' ')

if __name__ == '__main__': 
    main(sys.argv[1:])