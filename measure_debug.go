package decad

import (
	"context"
	"fmt"
	"math/big"
	"os"
	"runtime"
	"strings"
	"syscall"
	"time"
)

// Measurement-only code for the perf-s0-measure branch. Never merged.

func init() {
	model := ""
	if b, err := os.ReadFile("/proc/cpuinfo"); err == nil {
		for _, line := range strings.Split(string(b), "\n") {
			if strings.HasPrefix(line, "model name") {
				model = strings.TrimSpace(line)
				break
			}
		}
	}
	cg := "absent"
	if b, err := os.ReadFile("/sys/fs/cgroup/cpu.max"); err == nil {
		cg = strings.TrimSpace(string(b))
	}
	fmt.Fprintf(os.Stderr, "MEASURE cpu %s\n", model)
	fmt.Fprintf(os.Stderr, "MEASURE numcpu=%d gomaxprocs=%d\n", runtime.NumCPU(), runtime.GOMAXPROCS(0))
	fmt.Fprintf(os.Stderr, "MEASURE cgroup cpu.max=%s\n", cg)
}

func measureCPU() time.Duration {
	var ru syscall.Rusage
	if err := syscall.Getrusage(syscall.RUSAGE_SELF, &ru); err != nil {
		return 0
	}
	return time.Duration(ru.Utime.Nano() + ru.Stime.Nano())
}

var measureLastCPU = map[string]time.Duration{}

// measurePhase prints the wall time since start and the process CPU time
// spent since the previous call carrying the same name prefix group. The CPU
// delta is process-wide, so it is only meaningful for non-overlapping phases.
func measurePhase(name string, start time.Time) time.Time {
	now := time.Now()
	cpu := measureCPU()
	prev := measureLastCPU["last"]
	measureLastCPU["last"] = cpu
	fmt.Fprintf(os.Stderr, "MEASURE phase %s wall=%.3fs cpu_since_prev_mark=%.3fs\n",
		name, now.Sub(start).Seconds(), (cpu - prev).Seconds())
	return now
}

func measureCrossBoreProbe(ctx context.Context, rec *crossCylinderBoreRecord, setback float64, gap *big.Rat) {
	cell := func(i int) (crossBoreCellResult, error) { return crossBoreCellProof(rec, setback, i, gap) }
	run := func(lo, hi, workers int) time.Duration {
		start := time.Now()
		_, _ = crossBoreCellMaxima(ctx, hi-lo, workers, func(i int) (crossBoreCellResult, error) { return cell(lo + i) })
		return time.Since(start)
	}
	w1 := run(0, 256, 1)
	w4 := run(256, 512, 4)
	fmt.Fprintf(os.Stderr, "MEASURE probe cells0-255 workers=1 wall=%.3fs; cells256-511 workers=4 wall=%.3fs; speedup=%.2fx\n",
		w1.Seconds(), w4.Seconds(), w1.Seconds()/w4.Seconds())
}
