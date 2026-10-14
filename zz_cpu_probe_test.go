package decad

import (
	"fmt"
	"os"
	"runtime"
	"strings"
	"testing"
)

// cpuProbe prints the CPU model so a CI log shows which runner class a shard got.
func cpuProbe(t *testing.T, label string) {
	data, err := os.ReadFile("/proc/cpuinfo")
	if err != nil {
		t.Skip(err)
	}
	for _, line := range strings.Split(string(data), "\n") {
		if strings.HasPrefix(line, "model name") {
			fmt.Fprintf(os.Stderr, "CPUPROBE %s %s ncpu=%d\n", label, strings.TrimSpace(line), runtime.NumCPU())
			return
		}
	}
}

func TestZZCPUProbeShard0(t *testing.T)  { cpuProbe(t, "shard0") }
func TestZZCPUProbeShard1(t *testing.T)  { cpuProbe(t, "shard1") }
func TestZZCPUProbeShard8(t *testing.T)  { cpuProbe(t, "shard8") }
func TestZZCPUProbeShard10(t *testing.T) { cpuProbe(t, "shard10") }
