# FLASH Viterbi

## Overview

This repository contains two implementations of the Viterbi algorithm for finding the most likely sequence of hidden states (the Viterbi path) in a Hidden Markov Model (HMM):

1. `FLASH_Viterbi_multithread.c` - *FLASH_Viterbi*, a full-state space implementation using arrays. 
2. `FLASH_BS_Viterbi_multithread.c` - *FLASH_BS_Viterbi*, an extension of FLASH_Viterbi that incorporates dynamic Beam Search using min-heaps.

Both implementations use multithreading to parallelize the computation.

## Key Features

### Common Features (both implementations):

- Multithreaded processing using Windows API threads
- Dynamic task queue for workload distribution
- Log probabilities to avoid numerical underflow
- Support for large state spaces 
- Memory usage tracking and reporting

### FLASH Viterbi_multithread.c:

- Full state space exploration
- Uses arrays for probability storage
- Divides observation sequence into segments for parallel processing
- Reports total memory usage

### FLASH_BS Viterbi_multithread.c:

- State space pruning using min-heaps
- Maintains only the top N most probable states
- Heap-based probability management
- Reduced memory footprint compared to full array version
- Tracks both current state and backtracking information

## Usage

1. Prepare input files containing:

   - Transition matrix (A)
   - Emission matrix (B)
   - Initial state probabilities (pi)
   - Observation sequence

2. Update file paths in the code or modify to accept command line arguments

3. Compile and run:

   ```text
   gcc FLASH_Viterbi_multithread.c -o FLASH_Viterbi
   ./FLASH_Viterbi
   ```
   
   or
   
   ```text
   gcc FLASH_BS_Viterbi_multithread.c -o FLASH_BS_Viterbi
   ./FLASH_BS_Viterbi
   ```

## Data Generation

You can use the following scripts to generate synthetic datasets for testing the Viterbi implementations:

### 1. `example_script.py`: Random Sparse Graph-Based HMM

This script generates transition (`A`), emission (`B`), initial probability (`Pi`), and observation (`ob`) files using a sparse random graph.

**Usage:**

```bash
python example_script.py -s <seed> -n <n_ob> -K <K> -T <T> -b <beam_width> -p <prob>
```

**Arguments:**

- `-s`: Random seed
- `-n`: Number of distinct observation symbols
- `-K`: Number of hidden states
- `-T`: Length of observation sequence
- `-b`: Beam width (for reference logging)
- `-p`: Probability of edge existence between states

**Output Files:**

- `A_K{K}_T{T}_prob{prob}.txt` – Transition matrix
- `B_K{K}_T{T}_prob{prob}.txt` – Emission matrix
- `Pi_K{K}_T{T}_prob{prob}.txt` – Initial state probabilities
- `ob_K{K}_T{T}_prob{prob}.txt` – Observation sequence

---

### 2. `example_script_dag.py`: DAG-Based Transition Structure

This script constructs a directed acyclic graph (DAG) as the HMM transition structure for testing topologically ordered models.

**Usage:**

```bash
python example_script_dag.py -s <seed> -n <n_ob> -K <K> -T <T>
```

**Arguments:**

- `-s`: Random seed
- `-n`: Number of distinct observation symbols
- `-K`: Number of hidden states
- `-T`: Length of observation sequence

**Output Files:**

- `A_K{K}_T{T}_DAG.txt` – Transition matrix from a DAG
- `B_K{K}_T{T}_DAG.txt` – Emission matrix
- `Pi_K{K}_T{T}_DAG.txt` – Initial state probabilities
- `ob_K{K}_T{T}_DAG.txt` – Observation sequence

## Output

Both programs will:

1. Print memory usage statistics
2. Display the maximum path probability (in log and normal space)
3. Output the most likely hidden state sequence
4. Report execution time

## Configuration

Key parameters can be modified at the top of each file:

- `K_STATE` - Number of hidden states
- `T_STATE` - Number of observation symbols
- `ObserRouteLEN` - Length of observation sequence
- `BeamSearchWidth` - (Heap version only) Beam search width,Number of states to maintain
- `MAX_THREADS` - Number of worker threads

## Performance Notes

- The FLASH_BS_Viterbi implementation is more memory-efficient for large state spaces
- The FLASH_Viterbi implementation may be faster for small state spaces
- Actual performance depends on observation sequence length and available cores

## Requirements

- Windows OS (uses Windows threading API)
- C compiler with support for C99 features
