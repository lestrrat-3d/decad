package main

import (
	"crypto/sha256"
	"encoding/hex"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

const fixtureSum = `package p

// Sum adds two ints. Lean proofs depend on this function; see _proofs/trace.json.
func Sum(a, b int) int { return a + b }

// Lean proofs depend on this function; see _proofs/trace.json.
func (t *Box[T]) Get() int { return 1 }

type Box[T any] struct{}
`

const fixtureSumPlain = `//go:build plain

package p

// Lean proofs depend on this function; see _proofs/trace.json.
func Sum(a, b int) int { return b + a }
`

const fixtureLean = `module

namespace Decad.X

theorem sum_sound : True := trivial

lemma get_sound : True := trivial

end Decad.X
`

const fixtureDoc = `# Proofs

| Id | Assumption |
|---|---|
| A1 | float adds round to nearest |
| A2 | Sum is exact |
`

const fixtureWorkflow = `on:
  pull_request:
    paths:
      - 'internal/p/sum.go'
      - 'internal/p/sum_plain.go'
`

const fixtureManifest = `{
  "entries": [
    {
      "file": "internal/p/sum.go",
      "func": "Box.Get",
      "role": "models",
      "theorems": ["Decad/X.lean:Decad.X.get_sound"],
      "assumptions": [],
      "sha256": ""
    },
    {
      "file": "internal/p/sum.go",
      "func": "Sum",
      "role": "models",
      "theorems": ["Decad/X.lean:Decad.X.sum_sound"],
      "assumptions": ["A1"],
      "sha256": ""
    },
    {
      "file": "internal/p/sum_plain.go",
      "func": "Sum",
      "role": "assumes",
      "theorems": [],
      "assumptions": ["A2"],
      "sha256": ""
    }
  ]
}
`

// newRepo writes a fake repository whose manifest hashes are filled by Update.
func newRepo(t *testing.T) string {
	t.Helper()
	repo := t.TempDir()
	write(t, repo, "internal/p/sum.go", fixtureSum)
	write(t, repo, "internal/p/sum_plain.go", fixtureSumPlain)
	write(t, repo, "_proofs/Decad/X.lean", fixtureLean)
	write(t, repo, "_proofs/trace.json", fixtureManifest)
	write(t, repo, "agent-docs/proofs-design.md", fixtureDoc)
	write(t, repo, ".github/workflows/proofs.yml", fixtureWorkflow)
	if _, err := Update(repo); err != nil {
		t.Fatalf("Update: %v", err)
	}
	return repo
}

func write(t *testing.T, repo, rel, content string) {
	t.Helper()
	path := filepath.Join(repo, filepath.FromSlash(rel))
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
		t.Fatal(err)
	}
}

func read(t *testing.T, repo, rel string) string {
	t.Helper()
	data, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(rel)))
	if err != nil {
		t.Fatal(err)
	}
	return string(data)
}

func replace(t *testing.T, repo, rel, old, repl string) {
	t.Helper()
	src := read(t, repo, rel)
	if !strings.Contains(src, old) {
		t.Fatalf("%s does not contain %q", rel, old)
	}
	write(t, repo, rel, strings.Replace(src, old, repl, 1))
}

// wantFailure requires exactly one failure line and that it contains every part.
func wantFailure(t *testing.T, fails []string, parts ...string) {
	t.Helper()
	if len(fails) != 1 {
		t.Fatalf("want exactly 1 failure, got %d: %q", len(fails), fails)
	}
	for _, p := range parts {
		if !strings.Contains(fails[0], p) {
			t.Fatalf("failure %q lacks %q", fails[0], p)
		}
	}
}

func TestCleanPass(t *testing.T) {
	repo := newRepo(t)
	if fails := Check(repo); len(fails) != 0 {
		t.Fatalf("want no failures, got %q", fails)
	}
}

func TestHashCoversFuncKeywordThroughBrace(t *testing.T) {
	repo := newRepo(t)
	want := sha256.Sum256([]byte("func Sum(a, b int) int { return a + b }"))
	m, err := loadManifest(repo)
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range m.Entries {
		if e.File == "internal/p/sum.go" && e.Func == "Sum" {
			if e.SHA256 != hex.EncodeToString(want[:]) {
				t.Fatalf("hash %s, want %s", e.SHA256, hex.EncodeToString(want[:]))
			}
			return
		}
	}
	t.Fatal("entry not found")
}

func TestBodyEditFailsHashCheck(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "internal/p/sum.go", "return a + b", "return a - b")
	wantFailure(t, Check(repo), "internal/p/sum.go Sum", "source changed", "Decad/X.lean:Decad.X.sum_sound", "-update")
}

func TestDocCommentEditPasses(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "internal/p/sum.go", "Sum adds two ints.", "Sum returns the total of two ints.")
	if fails := Check(repo); len(fails) != 0 {
		t.Fatalf("want no failures, got %q", fails)
	}
}

func TestMissingFuncFails(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "internal/p/sum.go", "func Sum(", "func Total(")
	fails := Check(repo)
	// The renamed function also leaves a stray pointer on Total.
	if len(fails) != 2 {
		t.Fatalf("want 2 failures, got %q", fails)
	}
	if !strings.Contains(fails[0], "internal/p/sum.go Sum: function not found") {
		t.Fatalf("unexpected failure %q", fails[0])
	}
	if !strings.Contains(fails[1], "internal/p/sum.go Total") || !strings.Contains(fails[1], "no entry") {
		t.Fatalf("unexpected failure %q", fails[1])
	}
}

func TestBuildTagTwinResolvesToItsOwnFile(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "internal/p/sum_plain.go", "return b + a", "return b - a")
	wantFailure(t, Check(repo), "internal/p/sum_plain.go Sum", "source changed")
}

func TestUnknownTheoremFails(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "_proofs/Decad/X.lean", "theorem sum_sound", "theorem sum_renamed")
	wantFailure(t, Check(repo), "internal/p/sum.go Sum", "sum_sound", "not declared")
}

func TestMissingLeanFileFails(t *testing.T) {
	repo := newRepo(t)
	if err := os.Remove(filepath.Join(repo, "_proofs/Decad/X.lean")); err != nil {
		t.Fatal(err)
	}
	fails := Check(repo)
	if len(fails) != 2 {
		t.Fatalf("want one failure per theorem entry, got %q", fails)
	}
	if !strings.Contains(fails[0], "Lean file _proofs/Decad/X.lean not found") {
		t.Fatalf("unexpected failure %q", fails[0])
	}
}

func TestDanglingAssumptionFails(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "_proofs/trace.json", `"A1"`, `"A9"`)
	wantFailure(t, Check(repo), "internal/p/sum.go Sum", "A9", "agent-docs/proofs-design.md")
}

func TestMissingPointerFails(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "internal/p/sum_plain.go",
		"// Lean proofs depend on this function; see _proofs/trace.json.\n", "")
	wantFailure(t, Check(repo), "internal/p/sum_plain.go Sum", "lacks the pointer")
}

func TestStrayPointerFails(t *testing.T) {
	repo := newRepo(t)
	write(t, repo, "internal/q/extra.go", `package q

// Lean proofs depend on this function; see _proofs/trace.json.
func Extra() {}
`)
	wantFailure(t, Check(repo), "internal/q/extra.go Extra", "no entry")
}

func TestStrayPointerSkipsUnderscoreDotAndTestFiles(t *testing.T) {
	repo := newRepo(t)
	const stray = "package q\n\n// Lean proofs depend on this function; see _proofs/trace.json.\nfunc Extra() {}\n"
	write(t, repo, "_gallery/extra.go", stray)
	write(t, repo, ".hidden/extra.go", stray)
	write(t, repo, "internal/q/extra_test.go", stray)
	if fails := Check(repo); len(fails) != 0 {
		t.Fatalf("want no failures, got %q", fails)
	}
}

func TestMissingWorkflowPathFails(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, ".github/workflows/proofs.yml", "      - 'internal/p/sum_plain.go'\n", "")
	wantFailure(t, Check(repo), "internal/p/sum_plain.go Sum", "proofs.yml")
}

func TestUnsortedManifestFails(t *testing.T) {
	repo := newRepo(t)
	m, err := loadManifest(repo)
	if err != nil {
		t.Fatal(err)
	}
	m.Entries[0], m.Entries[1] = m.Entries[1], m.Entries[0]
	data, err := encodeManifest(m)
	if err != nil {
		t.Fatal(err)
	}
	write(t, repo, "_proofs/trace.json", string(data))
	wantFailure(t, Check(repo), "not sorted", "internal/p/sum.go Box.Get")
}

func TestDuplicateEntryFails(t *testing.T) {
	repo := newRepo(t)
	m, err := loadManifest(repo)
	if err != nil {
		t.Fatal(err)
	}
	m.Entries = append(m.Entries[:2], m.Entries[1:]...)
	data, err := encodeManifest(m)
	if err != nil {
		t.Fatal(err)
	}
	write(t, repo, "_proofs/trace.json", string(data))
	wantFailure(t, Check(repo), "duplicate entry")
}

func TestRoleRulesFail(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "_proofs/trace.json", `"Decad/X.lean:Decad.X.get_sound"`, ``)
	wantFailure(t, Check(repo), "internal/p/sum.go Box.Get", "needs at least one theorem")
}

func TestUpdateFixesOnlyHashes(t *testing.T) {
	repo := newRepo(t)
	before := read(t, repo, "_proofs/trace.json")
	replace(t, repo, "internal/p/sum.go", "return a + b", "return a - b")
	changes, err := Update(repo)
	if err != nil {
		t.Fatal(err)
	}
	if len(changes) != 1 || !strings.Contains(changes[0], "internal/p/sum.go Sum") {
		t.Fatalf("want one change for Sum, got %q", changes)
	}
	after := read(t, repo, "_proofs/trace.json")
	m, err := loadManifest(repo)
	if err != nil {
		t.Fatal(err)
	}
	if len(m.Entries) != 3 {
		t.Fatalf("Update changed the entry count to %d", len(m.Entries))
	}
	// Reverting the one hash line restores the original manifest byte for byte.
	var oldLine, newLine string
	for _, l := range strings.Split(before, "\n") {
		if strings.Contains(l, "sha256") && !strings.Contains(after, l) {
			oldLine = l
		}
	}
	for _, l := range strings.Split(after, "\n") {
		if strings.Contains(l, "sha256") && !strings.Contains(before, l) {
			newLine = l
		}
	}
	if oldLine == "" || newLine == "" || strings.Replace(after, newLine, oldLine, 1) != before {
		t.Fatalf("Update changed more than one sha256 line:\nbefore:\n%s\nafter:\n%s", before, after)
	}
	if fails := Check(repo); len(fails) != 0 {
		t.Fatalf("want no failures after Update, got %q", fails)
	}
}

func TestUpdateKeepsOtherFailures(t *testing.T) {
	repo := newRepo(t)
	replace(t, repo, "internal/p/sum.go", "func Sum(", "func Total(")
	changes, err := Update(repo)
	if err != nil {
		t.Fatal(err)
	}
	for _, c := range changes {
		if strings.Contains(c, "Sum") {
			t.Fatalf("Update touched the entry whose function is missing: %q", c)
		}
	}
	if fails := Check(repo); len(fails) == 0 {
		t.Fatal("want failures for the missing function")
	}
}

func TestUpdateNeverAddsEntries(t *testing.T) {
	repo := newRepo(t)
	write(t, repo, "internal/q/extra.go", `package q

// Lean proofs depend on this function; see _proofs/trace.json.
func Extra() {}
`)
	if _, err := Update(repo); err != nil {
		t.Fatal(err)
	}
	m, err := loadManifest(repo)
	if err != nil {
		t.Fatal(err)
	}
	if len(m.Entries) != 3 {
		t.Fatalf("Update added entries: %d", len(m.Entries))
	}
	wantFailure(t, Check(repo), "internal/q/extra.go Extra")
}
