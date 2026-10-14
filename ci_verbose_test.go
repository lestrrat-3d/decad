package decad

import (
	"flag"
	"fmt"
	"os"
	"runtime"
	"strings"
	"testing"
)

// TestMain turns on -test.v when GITHUB_ACTIONS is "true", so every CI shard
// log carries one "--- PASS: Name (x.xxs)" line per test and shard balancing
// can be measured from the logs. With any other value the test binary runs
// exactly as go test starts it.
//
// This must stay the package's only TestMain: Go allows one per test binary,
// so a test that needs setup must extend this function, not add a second.
func TestMain(m *testing.M) {
	flag.Parse()
	ci := os.Getenv("GITHUB_ACTIONS") == "true"
	if ci {
		if err := flag.Set("test.v", "true"); err != nil {
			panic(err)
		}
	}
	code := m.Run()
	if ci {
		memProbe()
	}
	os.Exit(code)
}

// memProbe prints peak and current process memory. Measurement only.
func memProbe() {
	var hwm, rss string
	if data, err := os.ReadFile("/proc/self/status"); err == nil {
		for _, line := range strings.Split(string(data), "\n") {
			f := strings.Fields(line)
			if len(f) < 2 {
				continue
			}
			switch f[0] {
			case "VmHWM:":
				hwm = f[1]
			case "VmRSS:":
				rss = f[1]
			}
		}
	}
	var ms runtime.MemStats
	runtime.ReadMemStats(&ms)
	fmt.Fprintf(os.Stderr, "MEMPROBE vmhwm_kb=%s vmrss_kb=%s goheapsys=%d gosys=%d\n", hwm, rss, ms.HeapSys, ms.Sys)
}
