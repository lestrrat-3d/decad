// Command tracecheck verifies _proofs/trace.json against the Go source, the
// Lean sources, the assumptions table and the proofs workflow. Run it from
// _proofs:
//
//	go run ./tracecheck            check; exit 1 on any failure
//	go run ./tracecheck -update    rewrite the sha256 of entries whose function exists, then check
//
// -update NEVER adds or removes entries. Run it only after re-reading each
// affected theorem against the changed Go code. agent-docs/proofs-design.md
// owns the manifest format and the checks.
package main

import (
	"flag"
	"fmt"
	"os"
)

func main() {
	os.Exit(run(os.Args[1:], os.Stdout, os.Stderr))
}

func run(args []string, stdout, stderr *os.File) int {
	fs := flag.NewFlagSet("tracecheck", flag.ContinueOnError)
	fs.SetOutput(stderr)
	repo := fs.String("repo", "..", "repository root")
	update := fs.Bool("update", false, "rewrite the sha256 of entries whose function is found")
	if err := fs.Parse(args); err != nil {
		return 2
	}

	if *update {
		changes, err := Update(*repo)
		if err != nil {
			fmt.Fprintln(stderr, err)
			return 1
		}
		for _, c := range changes {
			fmt.Fprintln(stdout, c)
		}
	}

	failures := Check(*repo)
	for _, f := range failures {
		fmt.Fprintln(stderr, f)
	}
	if len(failures) > 0 {
		return 1
	}
	return 0
}
