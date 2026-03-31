#!/bin/bash

# ==============================================================================
# PHASE 2: SHADOW ABLATION (FETCH FAIRNESS)
# Testing 2S2W Configuration with HARDCODED Fair (Round Robin) Fetch Policy
# ==============================================================================

# --- HARDWARE PARAMETERS (Identical to Phase 1) ---
ROBSize=128
numSIQEntries=128
numWIQEntries=10
LQEntries=72
SQEntries=68
numPhysFloatRegs=300
numPhysVecRegs=300
numPhysIntRegs=300

# --- OUTPUT DIRECTORY SETUP ---
OUT_BASE="../phase2_results"

echo "==========================================================="
echo " Creating Phase 2 Directory Structure in $OUT_BASE..."
echo "==========================================================="
mkdir -p $OUT_BASE/bfs/shadow_2S2W_fair
mkdir -p $OUT_BASE/bc/shadow_2S2W_fair

# ==============================================================================
# WORKLOAD 1: BREADTH-FIRST SEARCH (BFS)
# ==============================================================================
echo ""
echo ">>> [1/2] Running BFS: Phase 2 (Hardcoded Fair Fetch)..."
time build/ARM/gem5.opt -d $OUT_BASE/bfs/shadow_2S2W_fair configs/example/se_SMT_ARM.py \
    --cmd="bmrk_binaries/CRONO/bfs" \
    --options="4 graph4096.txt" \
    --threadTypes S \
    --cpu="o3_grace_test" \
    --mem-type=DDR4_2400_16x4 \
    --mem-size=2GB \
    --mem-channels=2 \
    --num-cpus=1 \
    --smt -t 4 -WThreads 2 -SThreads 2 \
    -FirstThreadSOtherW=True \
    -ROBSize $ROBSize -numSIQEntries $numSIQEntries -numWIQEntries $numWIQEntries -numIQEntries $numSIQEntries \
    -LQEntries $LQEntries -SQEntries $SQEntries \
    -numPhysFloatRegs $numPhysFloatRegs -numPhysVecRegs $numPhysVecRegs -numPhysIntRegs $numPhysIntRegs \
    > $OUT_BASE/bfs/shadow_2S2W_fair/stdout.txt 2> $OUT_BASE/bfs/shadow_2S2W_fair/stderr.txt


# ==============================================================================
# WORKLOAD 2: BETWEENNESS CENTRALITY (BC)
# ==============================================================================
echo ""
echo ">>> [2/2] Running BC: Phase 2 (Hardcoded Fair Fetch)..."
time build/ARM/gem5.opt -d $OUT_BASE/bc/shadow_2S2W_fair configs/example/se_SMT_ARM.py \
    --cmd="bmrk_binaries/CRONO/bc_work_stealing" \
    --options="4 128 16" \
    --threadTypes S \
    --cpu="o3_grace_test" \
    --mem-type=DDR4_2400_16x4 \
    --mem-size=2GB \
    --mem-channels=2 \
    --num-cpus=1 \
    --smt -t 4 -WThreads 2 -SThreads 2 \
    -FirstThreadSOtherW=True \
    -ROBSize $ROBSize -numSIQEntries $numSIQEntries -numWIQEntries $numWIQEntries -numIQEntries $numSIQEntries \
    -LQEntries $LQEntries -SQEntries $SQEntries \
    -numPhysFloatRegs $numPhysFloatRegs -numPhysVecRegs $numPhysVecRegs -numPhysIntRegs $numPhysIntRegs \
    > $OUT_BASE/bc/shadow_2S2W_fair/stdout.txt 2> $OUT_BASE/bc/shadow_2S2W_fair/stderr.txt


# ==============================================================================
# DATA EXTRACTION & SUMMARY
# ==============================================================================
echo ""
echo "==========================================================="
echo " PHASE 2 SIMULATIONS COMPLETE! "
echo "==========================================================="
echo "Extracting Main Thread (Thread 0) IPC for Comparison:"
echo "-----------------------------------------------------------"

echo "[BFS] Phase 1 SHADOW (Greedy) IPC: "
grep "system.cpu_cluster.cpus.ipc::0" ../phase1_results/bfs/shadow_2S2W/stats.txt || echo "Run Phase 1 first."

echo "[BFS] Phase 2 SHADOW (Fair) IPC:   "
grep "system.cpu_cluster.cpus.ipc::0" $OUT_BASE/bfs/shadow_2S2W_fair/stats.txt || echo "Error: Stat not found."

echo "-----------------------------------------------------------"
echo "[BC] Phase 1 SHADOW (Greedy) IPC:  "
grep "system.cpu_cluster.cpus.ipc::0" ../phase1_results/bc/shadow_2S2W/stats.txt || echo "Run Phase 1 first."

echo "[BC] Phase 2 SHADOW (Fair) IPC:    "
grep "system.cpu_cluster.cpus.ipc::0" $OUT_BASE/bc/shadow_2S2W_fair/stats.txt || echo "Error: Stat not found."
echo "==========================================================="