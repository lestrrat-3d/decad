package decad

import (
	"bufio"
	"fmt"
	"os"
	"runtime"
	"strings"
	"testing"
)

// cpuClassModelName returns the first "model name" value of the cpuinfo file
// at path, or "" when the file is unreadable or has no such line.
func cpuClassModelName(path string) string {
	f, err := os.Open(path)
	if err != nil {
		return ""
	}
	defer f.Close()

	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		if !strings.HasPrefix(line, "model name") {
			continue
		}
		_, v, ok := strings.Cut(line, ":")
		if !ok {
			continue
		}
		return strings.TrimSpace(v)
	}
	return ""
}

// logCICPUClass writes one CPUPROBE line naming the runner's CPU class to
// stderr, so each CI shard job log records it. Stderr is used because passing
// tests print nothing without -v and the shard binaries run without -v. It
// never fails or skips: where /proc/cpuinfo is unreadable or has no model
// name (arm64 Linux, macOS, Windows) it prints GOOS/GOARCH instead.
func logCICPUClass(label string) {
	model := cpuClassModelName("/proc/cpuinfo")
	if model == "" {
		model = runtime.GOOS + "/" + runtime.GOARCH
	}
	fmt.Fprintf(os.Stderr, "CPUPROBE %s %s ncpu=%d\n", label, model, runtime.NumCPU())
}

// The manifest assigns each test to exactly one shard and every shard must
// run one, so there is one tiny test per shard.

func TestCICPUClassShard0(t *testing.T)  { logCICPUClass("shard0") }
func TestCICPUClassShard1(t *testing.T)  { logCICPUClass("shard1") }
func TestCICPUClassShard2(t *testing.T)  { logCICPUClass("shard2") }
func TestCICPUClassShard3(t *testing.T)  { logCICPUClass("shard3") }
func TestCICPUClassShard4(t *testing.T)  { logCICPUClass("shard4") }
func TestCICPUClassShard5(t *testing.T)  { logCICPUClass("shard5") }
func TestCICPUClassShard6(t *testing.T)  { logCICPUClass("shard6") }
func TestCICPUClassShard7(t *testing.T)  { logCICPUClass("shard7") }
func TestCICPUClassShard8(t *testing.T)  { logCICPUClass("shard8") }
func TestCICPUClassShard9(t *testing.T)  { logCICPUClass("shard9") }
func TestCICPUClassShard10(t *testing.T) { logCICPUClass("shard10") }
