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

// option reports whether e is a call of a decad function in the qualifier
// form this file uses, and returns its name and the source text of its
// arguments (from the first to the last, with line breaks and comments, and
// the dots of a spread tail). spread reports a spread tail: a caller accepts
// one only for an option whose method takes anyArgs.
func (c *callSite) option(e ast.Expr) (name, argText string, args []ast.Expr, spread, ok bool) {
	call, isCall := e.(*ast.CallExpr)
	if !isCall {
		return "", "", nil, false, false
	}
	fname, qual := callee(call.Fun, c.pkg, c.pkg == "")
	if fname == "" || qual != c.pkg {
		return "", "", nil, false, false
	}
	if len(call.Args) > 0 {
		argText = c.spanOf(call.Args[0], call.Args[len(call.Args)-1])
	}
	if call.Ellipsis.IsValid() {
		// A spread tail, WithSectionScale(factors...), keeps its dots.
		argText += "..."
	}
	return fname, argText, call.Args, call.Ellipsis.IsValid(), true
}

// rule rewrites one function call. It returns the replacement text, or a
// reason it cannot.
type rule func(c *callSite) (string, string)

var families = map[string]map[string]rule{
	"modify": {
		"Fillet":       ruleModify("Fillet", 3, optionMethods{"WithTangentChain": {"TangentChain", 0}}),
		"Chamfer":      ruleModify("Chamfer", 3, optionMethods{"WithTangentChain": {"TangentChain", 0}, "WithAsymmetricChamfer": {"Asymmetric", 2}}),
		"FilletChord":  ruleModify("FilletChord", 3, nil),
		"ChamferAngle": ruleModify("ChamferAngle", 5, nil),
		"Draft":        ruleModify("Draft", 4, nil),
		"Offset":       ruleModify("Offset", 2, optionMethods{"WithOffsetSide": {"Side", 1}}),
		"Thicken":      ruleModify("Thicken", 2, optionMethods{"WithThickenSide": {"Side", 1}}),
		"MoveFace":     ruleModify("MoveFace", 3, nil),
		"ConeFromFace": ruleModify("ConeFromFace", 3, nil),
		"Shell":        ruleShell,
	},
	"features": {
		"Extrude": ruleModify("Extrude", 4, optionMethods{"WithTaper": {"Taper", 1}, "WithSurfaceResult": {"SurfaceResult", 0}}),
		"Revolve": ruleModify("Revolve", 5, optionMethods{"WithSurfaceResult": {"SurfaceResult", 0}}),
		"Sweep": ruleModify("Sweep", 4, optionMethods{
			"WithSweepTwist": {"Twist", 1}, "WithMitredJoins": {"MitredJoins", 0},
			"WithSectionScale": {"SectionScale", anyArgs}, "WithSurfaceResult": {"SurfaceResult", 0},
		}),
		"Coil": ruleModify("Coil", 6, optionMethods{"WithLeftHand": {"LeftHand", 0}}),
		"Loft": ruleModify("Loft", 5, optionMethods{
			"WithLoftAlignment": {"Alignment", anyArgs}, "WithLoftSplits": {"Splits", 2},
			"WithSurfaceResult": {"SurfaceResult", 0},
		}),
		"LoftChain":     ruleModify("LoftChain", 5, nil),
		"LoftFromPoint": ruleModify("LoftFromPoint", 4, nil),
		"PatchProfile":  ruleModify("PatchProfile", 3, nil),
		"Emboss": ruleModify("Emboss", 6, optionMethods{
			"WithEmbossRimFillet": {"RimFillet", 1}, "WithEmbossDraft": {"Draft", 1},
		}),
		"WebPath": ruleModify("WebPath", 5, optionMethods{
			"WithRibDraft": {"Draft", 1}, "WithRibOverrun": {"Overrun", 1},
		}),
		"CutExtrude":                             ruleModify("CutExtrude", 4, optionMethods{"WithTaper": {"Taper", 1}}),
		"CutRevolve":                             ruleModify("CutRevolve", 5, nil),
		"LoftSections":                           ruleLoftSections,
		"LoftSectionsWithCircularCorrespondence": ruleLoftSectionsCircular,
		"LoftFaces":                              ruleLoftFaces,
	},
	"placement":  placementRules,
	"composites": compositeRules,
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
	// A generic call spells its type argument, Extrude[*sketch.Profile](...). The
	// command constructor infers it from the section, so the rewrite drops it.
	if inst, ok := fun.(*ast.IndexExpr); ok {
		fun = inst.X
	}
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

// anyArgs is the argument count of a variadic option: the method takes
// whatever the option took.
const anyArgs = -1

// optionMethods maps an option constructor to the command method that
// replaces it and the number of arguments the option takes.
type optionMethods map[string]struct {
	method string
	nargs  int
}

// The option names the rules match are the pre-migration ones, such as
// WithTangentChain, because the tool migrates code written against the old
// API (agent-docs/command-api-design.md section 14).
//
// ruleModify: Fillet(ctx, b, sel, r, decad.WithTangentChain()) ->
// Fillet(b, sel, r).TangentChain().Do(ctx). mandatory counts the arguments
// after ctx that become constructor arguments; every later argument must be a
// call of one of the options in methods, with the argument count the method
// takes.
func ruleModify(name string, mandatory int, methods optionMethods) rule {
	return func(c *callSite) (string, string) {
		if c.ellipsis || len(c.args) < mandatory+1 {
			return "", name + " call with a spread option list or too few arguments"
		}
		call := fmt.Sprintf("%s(%s)", qualify(c.pkg, name), c.span(1, mandatory))
		chain, why := c.optionChain(name, c.args[mandatory+1:], methods)
		if why != "" {
			return "", why
		}
		return call + chain + ".Do(" + c.text(c.args[0]) + ")", ""
	}
}

// optionChain turns the option arguments into a method chain, or reports the
// first option it cannot map.
func (c *callSite) optionChain(name string, opts []ast.Expr, methods optionMethods) (string, string) {
	chain := ""
	for _, opt := range opts {
		oname, argText, args, spread, ok := c.option(opt)
		m, known := methods[oname]
		if !ok || !known || (m.nargs != anyArgs && (spread || len(args) != m.nargs)) {
			return "", name + " option " + c.text(opt) + " has no command method rule"
		}
		chain += "." + m.method + "(" + argText + ")"
	}
	return chain, ""
}

// ruleShell: Shell(ctx, b, sel, t, opts...) -> Shell(b, sel, t)...Do(ctx).
// WithNoOpenings() turns the call into ShellClosed(b, t), which takes no
// selector, so the selector argument must be the literal nil.
func ruleShell(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) < 4 {
		return "", "Shell call with a spread option list or too few arguments"
	}
	methods := optionMethods{"WithShellSense": {"Sense", 1}, "WithShellFaceThickness": {"FaceThickness", 2}}
	var rest []ast.Expr
	closed := false
	for _, opt := range c.args[4:] {
		if oname, _, args, spread, ok := c.option(opt); ok && oname == "WithNoOpenings" && len(args) == 0 && !spread {
			closed = true
			continue
		}
		rest = append(rest, opt)
	}
	chain, why := c.optionChain("Shell", rest, methods)
	if why != "" {
		return "", why
	}
	ctx := c.text(c.args[0])
	if !closed {
		return fmt.Sprintf("%s(%s)%s.Do(%s)", qualify(c.pkg, "Shell"), c.span(1, 3), chain, ctx), ""
	}
	if id, ok := c.args[2].(*ast.Ident); !ok || id.Name != "nil" {
		return "", "Shell with ShellClosed and a selector that is not the literal nil"
	}
	return fmt.Sprintf("%s(%s, %s)%s.Do(%s)", qualify(c.pkg, "ShellClosed"),
		c.text(c.args[1]), c.text(c.args[3]), chain, ctx), ""
}

// ruleLoftSections: LoftSections(ctx, d, sections...) ->
// LoftSections(d, sections...).Do(ctx). A spread slice keeps its ellipsis.
func ruleLoftSections(c *callSite) (string, string) {
	if len(c.args) < 2 {
		return "", "LoftSections call without a context and a document"
	}
	dots := ""
	if c.ellipsis {
		dots = "..."
	}
	return fmt.Sprintf("%s(%s%s).Do(%s)", qualify(c.pkg, "LoftSections"),
		c.span(1, len(c.args)-1), dots, c.text(c.args[0])), ""
}

// ruleLoftSectionsCircular: LoftSectionsWithCircularCorrespondence(ctx, d, c,
// sections...) -> LoftSections(d, sections...).CircularCorrespondence(c).Do(ctx).
func ruleLoftSectionsCircular(c *callSite) (string, string) {
	if len(c.args) < 3 {
		return "", "LoftSectionsWithCircularCorrespondence call without a context, a document and a correspondence"
	}
	sections := c.text(c.args[1])
	if len(c.args) > 3 {
		sections += ", " + c.span(3, len(c.args)-1)
	}
	if c.ellipsis {
		sections += "..."
	}
	return fmt.Sprintf("%s(%s).CircularCorrespondence(%s).Do(%s)", qualify(c.pkg, "LoftSections"),
		sections, c.text(c.args[2]), c.text(c.args[0])), ""
}

// ruleLoftFaces: LoftFaces(ctx, sections) -> LoftFaces(sections).Do(ctx), and
// LoftFaces(ctx, sections, WithConeTangentEnd(cone, radial)) ->
// LoftFacesToCone(sections, cone, radial).Do(ctx).
func ruleLoftFaces(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) < 2 {
		return "", "LoftFaces call with a spread option list or too few arguments"
	}
	ctx := c.text(c.args[0])
	switch len(c.args) {
	case 2:
		return fmt.Sprintf("%s(%s).Do(%s)", qualify(c.pkg, "LoftFaces"), c.text(c.args[1]), ctx), ""
	case 3:
		name, argText, args, spread, ok := c.option(c.args[2])
		if ok && name == "WithConeTangentEnd" && len(args) == 2 && !spread {
			return fmt.Sprintf("%s(%s, %s).Do(%s)", qualify(c.pkg, "LoftFacesToCone"),
				c.text(c.args[1]), argText, ctx), ""
		}
	}
	return "", "LoftFaces option " + c.text(c.args[len(c.args)-1]) + " has no command rule"
}
