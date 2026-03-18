#!/bin/bash

# ==============================================================================
# PHASE 1: SHADOW ARCHITECTURE RECREATION SCRIPT
# This script runs sequentially to prevent memory crashes (2GB limit).
# All results are piped to ../phase1_results/ to keep the repo clean.
# ==============================================================================

# --- HARDWARE PARAMETERS (Extracted for easy tweaking in Phase 2/3) ---
ROBSize=128
numSIQEntries=128
numWIQEntries=10
LQEntries=72
SQEntries=68
numPhysFloatRegs=300
numPhysVecRegs=300
numPhysIntRegs=300

# --- OUTPUT DIRECTORY SETUP ---
OUT_BASE="../phase1_results"

echo "==========================================================="
echo " Creating Phase 1 Directory Structure in $OUT_BASE..."
echo "==========================================================="
mkdir -p $OUT_BASE/bfs/baseline_SMT
mkdir -p $OUT_BASE/bfs/shadow_2S2W
mkdir -p $OUT_BASE/bc/baseline_SMT
mkdir -p $OUT_BASE/bc/shadow_2S2W


# ==============================================================================
# WORKLOAD 1: BREADTH-FIRST SEARCH (BFS)
# Arguments: <num_threads> <graph_file> -> "4 graph4096.txt"
# ==============================================================================
echo ""
echo ">>> [1/4] Running BFS: Baseline Standard SMT (2 Strong Threads)..."
time build/ARM/gem5.opt -d $OUT_BASE/bfs/baseline_SMT configs/example/se_SMT_ARM.py \
    --cmd="bmrk_binaries/CRONO/bfs" \
    --options="2 graph4096.txt" \
    --threadTypes S S \
    --cpu="o3_grace_test" \
    --mem-type=DDR4_2400_16x4 \
    --mem-size=2GB \
    --mem-channels=2 \
    --num-cpus=1 \
    --smt -t 2 -WThreads 0 -SThreads 2 \
    -FirstThreadSOtherW=False \
    -ROBSize $ROBSize -numSIQEntries $numSIQEntries -numWIQEntries $numWIQEntries -numIQEntries $numSIQEntries \
    -LQEntries $LQEntries -SQEntries $SQEntries \
    -numPhysFloatRegs $numPhysFloatRegs -numPhysVecRegs $numPhysVecRegs -numPhysIntRegs $numPhysIntRegs \
    > $OUT_BASE/bfs/baseline_SMT/stdout.txt 2> $OUT_BASE/bfs/baseline_SMT/stderr.txt

echo ">>> [2/4] Running BFS: SHADOW Asymmetric SMT (2 Strong, 2 Weak)..."
time build/ARM/gem5.opt -d $OUT_BASE/bfs/shadow_2S2W configs/example/se_SMT_ARM.py \
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
    > $OUT_BASE/bfs/shadow_2S2W/stdout.txt 2> $OUT_BASE/bfs/shadow_2S2W/stderr.txt


# ==============================================================================
# WORKLOAD 2: BETWEENNESS CENTRALITY (BC)
# Arguments: <num_threads> <nodes> <edges> -> "4 128 16"
# ==============================================================================
echo ""
echo ">>> [3/4] Running BC: Baseline Standard SMT (2 Strong Threads)..."
time build/ARM/gem5.opt -d $OUT_BASE/bc/baseline_SMT configs/example/se_SMT_ARM.py \
    --cmd="bmrk_binaries/CRONO/bc_work_stealing" \
    --options="2 128 16" \
    --threadTypes S S \
    --cpu="o3_grace_test" \
    --mem-type=DDR4_2400_16x4 \
    --mem-size=2GB \
    --mem-channels=2 \
    --num-cpus=1 \
    --smt -t 2 -WThreads 0 -SThreads 2 \
    -FirstThreadSOtherW=False \
    -ROBSize $ROBSize -numSIQEntries $numSIQEntries -numWIQEntries $numWIQEntries -numIQEntries $numSIQEntries \
    -LQEntries $LQEntries -SQEntries $SQEntries \
    -numPhysFloatRegs $numPhysFloatRegs -numPhysVecRegs $numPhysVecRegs -numPhysIntRegs $numPhysIntRegs \
    > $OUT_BASE/bc/baseline_SMT/stdout.txt 2> $OUT_BASE/bc/baseline_SMT/stderr.txt

echo ">>> [4/4] Running BC: SHADOW Asymmetric SMT (2 Strong, 2 Weak)..."
time build/ARM/gem5.opt -d $OUT_BASE/bc/shadow_2S2W configs/example/se_SMT_ARM.py \
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
    > $OUT_BASE/bc/shadow_2S2W/stdout.txt 2> $OUT_BASE/bc/shadow_2S2W/stderr.txt


# ==============================================================================
# DATA EXTRACTION & SUMMARY
# ==============================================================================
echo ""
echo "==========================================================="
echo " PHASE 1 SIMULATIONS COMPLETE! "
echo "==========================================================="
echo "Extracting Main Thread (Thread 0) IPC for Comparison:"
echo "-----------------------------------------------------------"

echo "[BFS] Baseline SMT IPC: "
grep "system.cpu_cluster.cpus.ipc::0" $OUT_BASE/bfs/baseline_SMT/stats.txt || echo "Error: Stat not found."

echo "[BFS] SHADOW (2S2W) IPC:"
grep "system.cpu_cluster.cpus.ipc::0" $OUT_BASE/bfs/shadow_2S2W/stats.txt || echo "Error: Stat not found."

echo "-----------------------------------------------------------"
echo "[BC] Baseline SMT IPC:  "
grep "system.cpu_cluster.cpus.ipc::0" $OUT_BASE/bc/baseline_SMT/stats.txt || echo "Error: Stat not found."

echo "[BC] SHADOW (2S2W) IPC: "
grep "system.cpu_cluster.cpus.ipc::0" $OUT_BASE/bc/shadow_2S2W/stats.txt || echo "Error: Stat not found."
echo "-----------------------------------------------------------"
echo "Check ../phase1_results/ for full stats.txt, stdout.txt, and stderr.txt files."