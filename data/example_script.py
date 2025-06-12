import numpy as np 
import random
import numpy as np
import sys,getopt

def create_A_b(n_nodes = 100, sd = 1, prob=0.2): 
    
    np.random.seed(sd) 
    matrix = np.zeros((n_nodes,n_nodes),dtype=float) 
    allstates = [x for x in range(n_nodes)]
    
    for state in range(n_nodes): 
        
        #edge_per_node=np.random.binomial(n_nodes-1, p=prob, size=None)
        edge_per_node=np.random.binomial(n_nodes, p=prob, size=None)
        
        #we sample @edge_per_node edges to connect to current state 
        #state_connections = np.random.choice(allstates, size=edge_per_node)
        state_connections = np.random.choice(allstates, size=edge_per_node, replace=False)
        
        #sample probabilities  
        ps = np.random.uniform(0.01,1, size=edge_per_node)
        
        for i in range(edge_per_node): 
            connection = state_connections[i]
            p = ps[i]
            matrix[state][connection] = p

    
    # normalize matrix 
    for i in range(n_nodes): 
        s = np.sum(matrix[i,]) 
        matrix[i,] = matrix[i,] / np.sum(matrix[i,]) 
       
                
    return matrix              
                

def create_B(n_observables = 100, n_states = 100, sd = 1): 
    
    ''' create matrix of uniform emission probabilities '''
    
    np.random.seed(sd) 
        
    #B = np.full((n_states,n_observables), float(np.random.rand(1)))
    B = np.random.uniform(0.1, 1, (n_states, n_observables))
    
    B = B/B.sum(axis=1)[:,None]
    
    return B 

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"-h-s:-n:-K:-T:-b:-p:")
    except getopt.GetoptError:
        print('example_script.py -s <seed> -n <n_ob> -K <K> -T <T> -b <beam_width> -p <prob>')
        sys.exit(2)
    num = 0
    for opt,arg in opts:
        if opt == '-h':
            print('example_script.py -s <seed> -n <n_ob> -K <K> -T <T> -b <beam_width> -p <prob>')
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
        elif opt in ("-b"):
            beam_width = int(arg)
            num += 1
        elif opt in ("-p"):
            prob = float(arg)
            num += 1
    if num != 6:
        print('example_script.py -s <seed> -n <n_ob> -K <K> -T <T> -b <beam_width> -p <prob>')
        sys.exit(2)
    file = open(f'ANS_K{K}_T{T}_prob{prob}_beam_width{beam_width}.txt','w')
    file.write(f"sd={sd}, n_ob={n_ob}, K={K}, T={T}, beam_width={beam_width}, prob={prob}\n")

    y = [random.randint(0,n_ob-1) for _ in range(T)] 
        
    # generate simple data 
    A = create_A_b(n_nodes = K, sd = sd,prob=prob)

    B = create_B(n_observables = n_ob,n_states = K, sd = sd)
        
    # uniform initial probabilities 
    pi = np.full(K, 1 / K)


    # Save A, B, Pi, y to txt files with float format
    np.savetxt(f'A_K{K}_T{T}_prob{prob}.txt', A, fmt='%.16f')
    np.savetxt(f'B_K{K}_T{T}_prob{prob}.txt', B, fmt='%.16f')
    np.savetxt(f'Pi_K{K}_T{T}_prob{prob}.txt', pi, fmt='%.16f', newline=' ')
    np.savetxt(f'ob_K{K}_T{T}_prob{prob}.txt', y, fmt='%d', newline=' ')
    

if __name__ == '__main__': 
    main(sys.argv[1:])