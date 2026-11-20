package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"os"
	"path/filepath"
	"regexp"
	"strings"
)

// pointerMarker is the text every covered function's doc comment carries.
const pointerMarker = "_proofs/trace.json"

// missingLean marks a Lean file that could not be read.
const missingLean = "\x00"

const (
	roleModels  = "models"
	roleAssumes = "assumes"
)

// Entry is one covered Go function. Key is (File, Func).
type Entry struct {
	File        string   `json:"file"`
	Func        string   `json:"func"`
	Role        string   `json:"role"`
	Theorems    []string `json:"theorems"`
	Assumptions []string `json:"assumptions"`
	SHA256      string   `json:"sha256"`
}

// Manifest is the content of _proofs/trace.json.
type Manifest struct {
	Entries []Entry `json:"entries"`
}

var (
	assumptionID = regexp.MustCompile(`^A[0-9]+$`)
	sha256Hex    = regexp.MustCompile(`^[0-9a-f]{64}$`)
)

func manifestPath(repo string) string { return filepath.Join(repo, "_proofs", "trace.json") }

func loadManifest(repo string) (Manifest, error) {
	var m Manifest
	path := manifestPath(repo)
	data, err := os.ReadFile(path)
	if err != nil {
		return m, fmt.Errorf("%s: cannot read manifest: %w", path, err)
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&m); err != nil {
		return m, fmt.Errorf("%s: manifest does not parse: %w", path, err)
	}
	return m, nil
}

func encodeManifest(m Manifest) ([]byte, error) {
	out := Manifest{Entries: make([]Entry, len(m.Entries))}
	for i, e := range m.Entries {
		if e.Theorems == nil {
			e.Theorems = []string{}
		}
		if e.Assumptions == nil {
			e.Assumptions = []string{}
		}
		out.Entries[i] = e
	}
	var buf bytes.Buffer
	enc := json.NewEncoder(&buf)
	enc.SetEscapeHTML(false)
	enc.SetIndent("", "  ")
	if err := enc.Encode(out); err != nil {
		return nil, err
	}
	return buf.Bytes(), nil
}

// goFile is one parsed Go source file with its function declarations.
type goFile struct {
	funcs []goFunc
	err   error
}

type goFunc struct {
	name string // Name, or Type.Name for a method
	hash string // sha256 of the source from the func keyword through the closing brace
	doc  string // doc comment text
}

// goFiles parses each repo file once per run.
type goFiles struct {
	repo  string
	files map[string]*goFile
}

func newGoFiles(repo string) *goFiles {
	return &goFiles{repo: repo, files: map[string]*goFile{}}
}

func (g *goFiles) load(rel string) *goFile {
	if f, ok := g.files[rel]; ok {
		return f
	}
	f := parseGoFile(filepath.Join(g.repo, filepath.FromSlash(rel)))
	g.files[rel] = f
	return f
}

// parseGoFile ignores build tags, so both files of a build-tag twin parse.
func parseGoFile(path string) *goFile {
	src, err := os.ReadFile(path)
	if err != nil {
		return &goFile{err: err}
	}
	fset := token.NewFileSet()
	af, err := parser.ParseFile(fset, path, src, parser.ParseComments)
	if err != nil {
		return &goFile{err: err}
	}
	var out goFile
	for _, d := range af.Decls {
		fd, ok := d.(*ast.FuncDecl)
		if !ok {
			continue
		}
		start := fset.Position(fd.Pos()).Offset
		end := fset.Position(fd.End()).Offset
		sum := sha256.Sum256(src[start:end])
		out.funcs = append(out.funcs, goFunc{
			name: declName(fd),
			hash: hex.EncodeToString(sum[:]),
			doc:  fd.Doc.Text(),
		})
	}
	return &out
}

func declName(fd *ast.FuncDecl) string {
	if fd.Recv == nil || len(fd.Recv.List) == 0 {
		return fd.Name.Name
	}
	return recvTypeName(fd.Recv.List[0].Type) + "." + fd.Name.Name
}

func recvTypeName(e ast.Expr) string {
	switch t := e.(type) {
	case *ast.StarExpr:
		return recvTypeName(t.X)
	case *ast.IndexExpr:
		return recvTypeName(t.X)
	case *ast.IndexListExpr:
		return recvTypeName(t.X)
	case *ast.ParenExpr:
		return recvTypeName(t.X)
	case *ast.Ident:
		return t.Name
	}
	return "?"
}

// lookup returns the function named fn, or why it cannot be used.
func (g *goFiles) lookup(rel, fn string) (goFunc, error) {
	f := g.load(rel)
	if f.err != nil {
		if errors.Is(f.err, fs.ErrNotExist) {
			return goFunc{}, errors.New("file not found; fix the entry's file or remove the entry")
		}
		return goFunc{}, fmt.Errorf("file does not parse: %v", f.err)
	}
	var found []goFunc
	for _, fn2 := range f.funcs {
		if fn2.name == fn {
			found = append(found, fn2)
		}
	}
	switch len(found) {
	case 0:
		return goFunc{}, errors.New("function not found; rename the entry to the new name or remove the entry")
	case 1:
		return found[0], nil
	}
	return goFunc{}, fmt.Errorf("function declared %d times; the entry key must name one", len(found))
}

// Update rewrites the sha256 of every entry whose function is found and
// returns one line per changed entry. It never adds or removes entries.
func Update(repo string) ([]string, error) {
	m, err := loadManifest(repo)
	if err != nil {
		return nil, err
	}
	g := newGoFiles(repo)
	var changes []string
	for i := range m.Entries {
		e := &m.Entries[i]
		fn, err := g.lookup(e.File, e.Func)
		if err != nil || fn.hash == e.SHA256 {
			continue
		}
		changes = append(changes, fmt.Sprintf("%s %s: sha256 %s -> %s", e.File, e.Func, short(e.SHA256), short(fn.hash)))
		e.SHA256 = fn.hash
	}
	if len(changes) == 0 {
		return nil, nil
	}
	data, err := encodeManifest(m)
	if err != nil {
		return nil, err
	}
	if err := os.WriteFile(manifestPath(repo), data, 0o644); err != nil {
		return nil, err
	}
	return changes, nil
}

func short(h string) string {
	if len(h) > 12 {
		return h[:12]
	}
	return h
}

// Check runs every check against the repository and returns one line per
// failure. An empty result means all checks passed.
func Check(repo string) []string {
	m, err := loadManifest(repo)
	if err != nil {
		return []string{err.Error()}
	}
	var fails []string
	fails = append(fails, checkStructure(m)...)
	g := newGoFiles(repo)
	fails = append(fails, checkSources(g, m)...)
	fails = append(fails, checkTheorems(repo, m)...)
	fails = append(fails, checkAssumptions(repo, m)...)
	fails = append(fails, checkWorkflow(repo, m)...)
	fails = append(fails, checkReverse(repo, m)...)
	return fails
}

func label(e Entry) string { return e.File + " " + e.Func }

// checkStructure is check 1: fields valid, sorted, no duplicate keys.
func checkStructure(m Manifest) []string {
	var fails []string
	for i, e := range m.Entries {
		l := label(e)
		if e.File == "" || e.Func == "" {
			fails = append(fails, fmt.Sprintf("entry %d: file and func are required; fill both or remove the entry", i))
			continue
		}
		switch e.Role {
		case roleModels:
			if len(e.Theorems) == 0 {
				fails = append(fails, l+": role models needs at least one theorem; add one or use role assumes")
			}
		case roleAssumes:
			if len(e.Assumptions) == 0 {
				fails = append(fails, l+": role assumes needs at least one assumption; add its A<n> id")
			}
		default:
			fails = append(fails, fmt.Sprintf("%s: role %q is not models or assumes; fix the role", l, e.Role))
		}
		for _, t := range e.Theorems {
			if _, _, ok := splitTheorem(t); !ok {
				fails = append(fails, fmt.Sprintf("%s: theorem %q is not <lean file>:<qualified name>; fix the entry", l, t))
			}
		}
		for _, a := range e.Assumptions {
			if !assumptionID.MatchString(a) {
				fails = append(fails, fmt.Sprintf("%s: assumption %q is not an A<n> id; fix the entry", l, a))
			}
		}
		if !sha256Hex.MatchString(e.SHA256) {
			fails = append(fails, l+": sha256 is not 64 lowercase hex digits; run go run ./tracecheck -update")
		}
		if i == 0 {
			continue
		}
		p := m.Entries[i-1]
		switch {
		case p.File == e.File && p.Func == e.Func:
			fails = append(fails, l+": duplicate entry; remove one")
		case p.File > e.File || (p.File == e.File && p.Func > e.Func):
			fails = append(fails, l+": entries are not sorted by (file, func); move it after "+label(p))
		}
	}
	return fails
}

// checkSources is checks 2 and 5: the function exists once, its hash matches
// and its doc comment carries the pointer.
func checkSources(g *goFiles, m Manifest) []string {
	var fails []string
	for _, e := range m.Entries {
		if e.File == "" || e.Func == "" {
			continue
		}
		l := label(e)
		fn, err := g.lookup(e.File, e.Func)
		if err != nil {
			fails = append(fails, l+": "+err.Error())
			continue
		}
		if fn.hash != e.SHA256 && sha256Hex.MatchString(e.SHA256) {
			fails = append(fails, fmt.Sprintf(
				"%s: source changed; re-check %s against the new code, then run go run ./tracecheck -update",
				l, theoremList(e)))
		}
		if !strings.Contains(fn.doc, pointerMarker) {
			fails = append(fails, fmt.Sprintf(
				"%s: doc comment lacks the pointer; add the line \"// Lean proofs depend on this function; see %s.\"",
				l, pointerMarker))
		}
	}
	return fails
}

func theoremList(e Entry) string {
	if len(e.Theorems) == 0 {
		return "the assumption rows " + strings.Join(e.Assumptions, ", ")
	}
	return strings.Join(e.Theorems, ", ")
}

func splitTheorem(t string) (file, name string, ok bool) {
	file, name, ok = strings.Cut(t, ":")
	return file, name, ok && file != "" && name != ""
}

// checkTheorems is check 3: each theorem's Lean file has a declaration line.
func checkTheorems(repo string, m Manifest) []string {
	var fails []string
	lean := map[string]string{}
	for _, e := range m.Entries {
		for _, t := range e.Theorems {
			file, name, ok := splitTheorem(t)
			if !ok {
				continue // reported by check 1
			}
			src, seen := lean[file]
			if !seen {
				data, err := os.ReadFile(filepath.Join(repo, "_proofs", filepath.FromSlash(file)))
				if err != nil {
					src = missingLean
				} else {
					src = string(data)
				}
				lean[file] = src
			}
			if src == missingLean {
				fails = append(fails, fmt.Sprintf("%s: Lean file _proofs/%s not found; fix the theorem path", label(e), file))
				continue
			}
			last := name[strings.LastIndex(name, ".")+1:]
			re := regexp.MustCompile(`(?m)^\s*(theorem|lemma)\s+` + regexp.QuoteMeta(last) + `\b`)
			if !re.MatchString(src) {
				fails = append(fails, fmt.Sprintf(
					"%s: theorem %s not declared in _proofs/%s; fix the name or restore the theorem", label(e), last, file))
			}
		}
	}
	return fails
}

// checkAssumptions is check 4: each assumption id has a row in the design doc.
func checkAssumptions(repo string, m Manifest) []string {
	docRel := "agent-docs/proofs-design.md"
	data, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(docRel)))
	var fails []string
	if err != nil {
		return []string{docRel + ": cannot read the assumptions table; restore the file"}
	}
	doc := string(data)
	for _, e := range m.Entries {
		for _, a := range e.Assumptions {
			if !assumptionID.MatchString(a) {
				continue // reported by check 1
			}
			if !regexp.MustCompile(`(?m)^\| ` + a + ` \|`).MatchString(doc) {
				fails = append(fails, fmt.Sprintf(
					"%s: assumption %s has no row in %s; add the row or fix the entry", label(e), a, docRel))
			}
		}
	}
	return fails
}

// checkWorkflow is check 7: each covered file is in the workflow's path filter.
func checkWorkflow(repo string, m Manifest) []string {
	wfRel := ".github/workflows/proofs.yml"
	data, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(wfRel)))
	if err != nil {
		return []string{wfRel + ": cannot read the workflow; restore the file"}
	}
	wf := string(data)
	var fails []string
	seen := map[string]struct{}{}
	for _, e := range m.Entries {
		if e.File == "" {
			continue
		}
		if _, ok := seen[e.File]; ok {
			continue
		}
		seen[e.File] = struct{}{}
		if !strings.Contains(wf, "'"+e.File+"'") {
			fails = append(fails, fmt.Sprintf(
				"%s %s: '%s' is missing from %s; add it to the pull_request and push path lists",
				e.File, e.Func, e.File, wfRel))
		}
	}
	return fails
}

// checkReverse is check 6: every function whose doc comment carries the
// pointer is a manifest entry. It skips dot- and underscore-prefixed
// directories and _test.go files.
func checkReverse(repo string, m Manifest) []string {
	have := map[[2]string]struct{}{}
	for _, e := range m.Entries {
		have[[2]string{e.File, e.Func}] = struct{}{}
	}
	var fails []string
	walkErr := filepath.WalkDir(repo, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		name := d.Name()
		if d.IsDir() {
			if path != repo && (strings.HasPrefix(name, ".") || strings.HasPrefix(name, "_")) {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(name, ".go") || strings.HasSuffix(name, "_test.go") {
			return nil
		}
		src, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		if !bytes.Contains(src, []byte(pointerMarker)) {
			return nil
		}
		rel, err := filepath.Rel(repo, path)
		if err != nil {
			return err
		}
		rel = filepath.ToSlash(rel)
		f := parseGoFile(path)
		if f.err != nil {
			fails = append(fails, fmt.Sprintf("%s: file does not parse: %v", rel, f.err))
			return nil
		}
		for _, fn := range f.funcs {
			if !strings.Contains(fn.doc, pointerMarker) {
				continue
			}
			if _, ok := have[[2]string{rel, fn.name}]; !ok {
				fails = append(fails, fmt.Sprintf(
					"%s %s: doc comment points to %s but the manifest has no entry; add the entry or remove the pointer line",
					rel, fn.name, pointerMarker))
			}
		}
		return nil
	})
	if walkErr != nil {
		fails = append(fails, "walking the repository: "+walkErr.Error())
	}
	return fails
}
