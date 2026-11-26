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

// rule rewrites one function call. args holds the source text of each call
// argument, ctx first. pkg is the qualifier to prefix a decad name with, "" in
// package decad. It returns the replacement text, or a reason it cannot.
type rule func(pkg string, args []string, ellipsis bool) (string, string)

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
func ruleUnion(pkg string, args []string, ellipsis bool) (string, string) {
	if len(args) < 2 {
		return "", "Union call without a context and a body"
	}
	ctx, rest := args[0], args[1:]
	if ellipsis {
		if len(rest) != 2 {
			return "", "Union with a spread slice and extra operands"
		}
		bodyType := qualify(pkg, "Body")
		return fmt.Sprintf("%s(append([]*%s{%s}, %s...)...).Do(%s)",
			qualify(pkg, "Union"), bodyType, rest[0], rest[1], ctx), ""
	}
	return fmt.Sprintf("%s(%s).Do(%s)", qualify(pkg, "Union"), strings.Join(rest, ", "), ctx), ""
}

// ruleBinary: Cut(ctx, a, b, decad.WithMatchingFaces()) ->
// Cut(a, b).MatchingFaces().Do(ctx).
func ruleBinary(name string) rule {
	return func(pkg string, args []string, ellipsis bool) (string, string) {
		if ellipsis || len(args) < 3 {
			return "", name + " call with a spread option list or too few arguments"
		}
		call := fmt.Sprintf("%s(%s, %s)", qualify(pkg, name), args[1], args[2])
		for _, opt := range args[3:] {
			if opt != qualify(pkg, "WithMatchingFaces")+"()" {
				return "", name + " option " + opt + " is not WithMatchingFaces()"
			}
			call += ".MatchingFaces()"
		}
		return call + ".Do(" + args[0] + ")", ""
	}
}

// ruleRefine: Refine(ctx, s, b) -> Refine(s, b).Do(ctx).
func ruleRefine(pkg string, args []string, ellipsis bool) (string, string) {
	if ellipsis || len(args) != 3 {
		return "", "Refine call without exactly a context, a body and a bound"
	}
	return fmt.Sprintf("%s(%s, %s).Do(%s)", qualify(pkg, "Refine"), args[1], args[2], args[0]), ""
}

// ruleUnionRefined: UnionRefined(ctx, bodies, bound) ->
// Union(bodies...).RefineTo(bound).Do(ctx). A slice literal spreads into its
// elements.
func ruleUnionRefined(pkg string, args []string, ellipsis bool) (string, string) {
	if ellipsis || len(args) != 3 {
		return "", "UnionRefined call without exactly a context, a slice and a bound"
	}
	bodies := args[1]
	if strings.HasPrefix(bodies, "[]*") && strings.HasSuffix(bodies, "}") {
		if open := strings.Index(bodies, "{"); open > 0 {
			bodies = strings.TrimSpace(bodies[open+1 : len(bodies)-1])
		}
	} else {
		bodies += "..."
	}
	return fmt.Sprintf("%s(%s).RefineTo(%s).Do(%s)", qualify(pkg, "Union"), bodies, args[2], args[0]), ""
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
			args := make([]string, len(call.Args))
			for i, a := range call.Args {
				args[i] = string(src[fset.Position(a.Pos()).Offset:fset.Position(a.End()).Offset])
			}
			text, why := r(qual, args, call.Ellipsis.IsValid())
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
