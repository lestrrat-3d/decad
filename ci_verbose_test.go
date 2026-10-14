package decad

import (
	"flag"
	"os"
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
	if os.Getenv("GITHUB_ACTIONS") == "true" {
		if err := flag.Set("test.v", "true"); err != nil {
			panic(err)
		}
	}
	os.Exit(m.Run())
}
