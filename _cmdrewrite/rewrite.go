package main

import (
	"fmt"
	"go/ast"
	"go/format"
	"go/parser"
	"go/token"
	"sort"
	"strconv"
	"strings"
)

const decadPath = "github.com/lestrrat-3d/decad"

// callSite is one call a rule may rewrite. args holds the call's argument
// expressions, ctx first, so a rule can match an option by its syntax tree
// (a name and its own arguments) instead of by its text. pkg is the qualifier
// to prefix a decad name with, "" in package decad.
type callSite struct {
	pkg      string
	args     []ast.Expr
	ellipsis bool
	src      []byte
	fset     *token.FileSet
}

// text returns the source text of node, with its line breaks.
func (c *callSite) text(node ast.Node) string {
	return string(c.src[c.fset.Position(node.Pos()).Offset:c.fset.Position(node.End()).Offset])
}

// span returns the source text from the start of args[from] to the end of
// args[to], inclusive, so a multi-line argument list keeps its line breaks
// and comments.
func (c *callSite) span(from, to int) string {
	return c.spanOf(c.args[from], c.args[to])
}

func (c *callSite) spanOf(first, last ast.Expr) string {
	return string(c.src[c.fset.Position(first.Pos()).Offset:c.fset.Position(last.End()).Offset])
}

// isCall reports whether e is a call of the decad function name with no
// arguments, in the qualifier form this file uses.
func (c *callSite) isCall(e ast.Expr, name string) bool {
	call, ok := e.(*ast.CallExpr)
	if !ok || len(call.Args) != 0 {
		return false
	}
	fname, qual := callee(call.Fun, c.pkg, c.pkg == "")
	return fname == name && qual == c.pkg
}

// rule rewrites one function call. It returns the replacement text, or a
// reason it cannot.
type rule func(c *callSite) (string, string)

var families = map[string]map[string]rule{
	"booleans": {
		"Union":        ruleUnion,
		"Cut":          ruleBinary("Cut"),
		"Intersect":    ruleBinary("Intersect"),
		"Refine":       ruleRefine,
		"UnionRefined": ruleUnionRefined,
	},
}

func familyNames() []string {
	var names []string
	for n := range families {
		names = append(names, n)
	}
	sort.Strings(names)
	return names
}

func qualify(pkg, name string) string {
	if pkg == "" {
		return name
	}
	return pkg + "." + name
}

// ruleUnion: Union(ctx, first, others...) -> Union(first, others...).Do(ctx).
func ruleUnion(c *callSite) (string, string) {
	if len(c.args) < 2 {
		return "", "Union call without a context and a body"
	}
	ctx := c.text(c.args[0])
	if c.ellipsis {
		if len(c.args) != 3 {
			return "", "Union with a spread slice and extra operands"
		}
		return fmt.Sprintf("%s(append([]*%s{%s}, %s...)...).Do(%s)",
			qualify(c.pkg, "Union"), qualify(c.pkg, "Body"), c.text(c.args[1]), c.text(c.args[2]), ctx), ""
	}
	return fmt.Sprintf("%s(%s).Do(%s)", qualify(c.pkg, "Union"), c.span(1, len(c.args)-1), ctx), ""
}

// ruleBinary: Cut(ctx, a, b, decad.WithMatchingFaces()) ->
// Cut(a, b).MatchingFaces().Do(ctx). Each option is matched by its syntax
// tree, so a later family adds a table row per option, with arguments where
// the option has them.
func ruleBinary(name string) rule {
	return func(c *callSite) (string, string) {
		if c.ellipsis || len(c.args) < 3 {
			return "", name + " call with a spread option list or too few arguments"
		}
		call := fmt.Sprintf("%s(%s)", qualify(c.pkg, name), c.span(1, 2))
		for _, opt := range c.args[3:] {
			if !c.isCall(opt, "WithMatchingFaces") {
				return "", name + " option " + c.text(opt) + " is not WithMatchingFaces()"
			}
			call += ".MatchingFaces()"
		}
		return call + ".Do(" + c.text(c.args[0]) + ")", ""
	}
}

// ruleRefine: Refine(ctx, s, b) -> Refine(s, b).Do(ctx).
func ruleRefine(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) != 3 {
		return "", "Refine call without exactly a context, a body and a bound"
	}
	return fmt.Sprintf("%s(%s).Do(%s)", qualify(c.pkg, "Refine"), c.span(1, 2), c.text(c.args[0])), ""
}

// ruleUnionRefined: UnionRefined(ctx, bodies, bound) ->
// Union(bodies...).RefineTo(bound).Do(ctx). A slice literal spreads into its
// elements.
func ruleUnionRefined(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) != 3 {
		return "", "UnionRefined call without exactly a context, a slice and a bound"
	}
	bodies := c.text(c.args[1]) + "..."
	if lit, ok := c.args[1].(*ast.CompositeLit); ok {
		bodies = ""
		if len(lit.Elts) > 0 {
			bodies = c.spanOf(lit.Elts[0], lit.Elts[len(lit.Elts)-1])
		}
	}
	return fmt.Sprintf("%s(%s).RefineTo(%s).Do(%s)",
		qualify(c.pkg, "Union"), bodies, c.text(c.args[2]), c.text(c.args[0])), ""
}

// rewriteFile applies rules to every matching call in src, innermost first,
// and returns the gofmt-formatted result with a report per call it skipped.
func rewriteFile(name string, src []byte, rules map[string]rule, root bool) ([]byte, []string, error) {
	var reports []string
	seen := map[string]struct{}{}
	for pass := 0; pass < 64; pass++ {
		fset := token.NewFileSet()
		file, err := parser.ParseFile(fset, name, src, parser.ParseComments)
		if err != nil {
			return nil, nil, err
		}
		pkgName := ""
		for _, imp := range file.Imports {
			path, _ := strconv.Unquote(imp.Path.Value)
			if path != decadPath {
				continue
			}
			pkgName = "decad"
			if imp.Name != nil {
				pkgName = imp.Name.Name
			}
		}
		bare := root && file.Name.Name == "decad"
		// A call that is the receiver of a command method is already in command
		// form; the old form is never a receiver.
		done := map[*ast.CallExpr]struct{}{}
		ast.Inspect(file, func(n ast.Node) bool {
			call, ok := n.(*ast.CallExpr)
			if !ok {
				return true
			}
			for {
				sel, ok := call.Fun.(*ast.SelectorExpr)
				if !ok {
					break
				}
				recv, ok := sel.X.(*ast.CallExpr)
				if !ok {
					break
				}
				done[recv] = struct{}{}
				call = recv
			}
			return true
		})
		type edit struct {
			start, end int
			text       string
		}
		var edits []edit
		var visit func(n ast.Node) bool
		visit = func(n ast.Node) bool {
			call, ok := n.(*ast.CallExpr)
			if !ok {
				return true
			}
			if _, converted := done[call]; converted {
				return true
			}
			fname, qual := callee(call.Fun, pkgName, bare)
			r, ok := rules[fname]
			if !ok {
				return true
			}
			// Rewrite the innermost matching calls first: descend, and only
			// rewrite this call when no call below it still matches.
			inner := false
			for _, arg := range call.Args {
				ast.Inspect(arg, func(m ast.Node) bool {
					if c, ok := m.(*ast.CallExpr); ok {
						if _, converted := done[c]; converted {
							return true
						}
						if f, _ := callee(c.Fun, pkgName, bare); rules[f] != nil {
							inner = true
						}
					}
					return !inner
				})
			}
			if inner {
				return true
			}
			text, why := r(&callSite{
				pkg: qual, args: call.Args, ellipsis: call.Ellipsis.IsValid(), src: src, fset: fset,
			})
			if why == "" && bare && !strings.HasSuffix(name, "_test.go") {
				// Design §11: a root composite calls the unexported implementation,
				// never a public command, so the author moves it by hand.
				why = "bare call in a root production file; call the unexported implementation instead"
			}
			start := fset.Position(call.Pos()).Offset
			if why != "" {
				key := fmt.Sprintf("%d:%s", start, why)
				if _, dup := seen[key]; !dup {
					seen[key] = struct{}{}
					reports = append(reports, fmt.Sprintf("%s: %s", fset.Position(call.Pos()), why))
				}
				return true
			}
			edits = append(edits, edit{start, fset.Position(call.End()).Offset, text})
			return false
		}
		ast.Inspect(file, visit)
		if len(edits) == 0 {
			break
		}
		sort.Slice(edits, func(i, j int) bool { return edits[i].start > edits[j].start })
		for _, e := range edits {
			src = append(append(append([]byte{}, src[:e.start]...), e.text...), src[e.end:]...)
		}
	}
	out, err := format.Source(src)
	if err != nil {
		return nil, nil, err
	}
	return out, reports, nil
}

// callee names the decad function a call expression calls and the qualifier
// it uses, or "" when the callee is no decad function.
func callee(fun ast.Expr, pkgName string, bare bool) (string, string) {
	switch f := fun.(type) {
	case *ast.SelectorExpr:
		if id, ok := f.X.(*ast.Ident); ok && pkgName != "" && id.Name == pkgName {
			return f.Sel.Name, pkgName
		}
	case *ast.Ident:
		if bare {
			return f.Name, ""
		}
	}
	return "", ""
}
