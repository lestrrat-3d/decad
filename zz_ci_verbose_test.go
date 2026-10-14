package decad

import (
	"flag"
	"os"
	"testing"
)

// Measurement only; never merged. On GitHub Actions it turns on -test.v so
// each shard log carries every test's duration. Elsewhere it does nothing.
func TestMain(m *testing.M) {
	flag.Parse()
	if os.Getenv("GITHUB_ACTIONS") == "true" {
		if err := flag.Set("test.v", "true"); err != nil {
			panic(err)
		}
	}
	os.Exit(m.Run())
}
