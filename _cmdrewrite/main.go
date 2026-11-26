// Command cmdrewrite rewrites call sites of decad's function-plus-options
// operations to the command form of agent-docs/command-api-design.md.
//
//	go run ./_cmdrewrite -family booleans [-root] <file-or-dir>...
//
// It matches calls by syntax: a selector call on the local name of the decad
// import, or, with -root, a bare call in a file of package decad. It cannot
// use go/types because the tree does not compile mid-migration, which is also
// why gofmt -r cannot do the job: a call with a variadic or option tail has no
// fixed shape. A call it cannot rewrite is reported and left as it is.
package main

import (
	"flag"
	"fmt"
	"io/fs"
	"os"
	"path/filepath"
	"strings"
)

func main() {
	family := flag.String("family", "", "family to rewrite: "+strings.Join(familyNames(), ", "))
	root := flag.Bool("root", false, "also rewrite bare calls in files of package decad")
	flag.Parse()
	rules, ok := families[*family]
	if !ok {
		fmt.Fprintf(os.Stderr, "cmdrewrite: unknown family %q\n", *family)
		os.Exit(2)
	}
	var files []string
	for _, arg := range flag.Args() {
		err := filepath.WalkDir(arg, func(path string, d fs.DirEntry, err error) error {
			if err != nil {
				return err
			}
			if d.IsDir() && (d.Name() == ".git" || d.Name() == ".tmp" || d.Name() == "testdata") {
				return filepath.SkipDir
			}
			if !d.IsDir() && strings.HasSuffix(path, ".go") {
				files = append(files, path)
			}
			return nil
		})
		if err != nil {
			fmt.Fprintf(os.Stderr, "cmdrewrite: %s\n", err)
			os.Exit(1)
		}
	}
	changed, failed := 0, 0
	for _, path := range files {
		src, err := os.ReadFile(path)
		if err != nil {
			fmt.Fprintf(os.Stderr, "cmdrewrite: %s\n", err)
			os.Exit(1)
		}
		out, reports, err := rewriteFile(path, src, rules, *root)
		if err != nil {
			fmt.Fprintf(os.Stderr, "cmdrewrite: %s: %s\n", path, err)
			failed++
			continue
		}
		for _, r := range reports {
			fmt.Printf("%s: %s\n", path, r)
			failed++
		}
		if string(out) != string(src) {
			if err := os.WriteFile(path, out, 0o644); err != nil {
				fmt.Fprintf(os.Stderr, "cmdrewrite: %s\n", err)
				os.Exit(1)
			}
			changed++
		}
	}
	fmt.Printf("cmdrewrite: %d files changed, %d reports\n", changed, failed)
	if failed > 0 {
		os.Exit(1)
	}
}
