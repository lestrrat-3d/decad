package main

import "fmt"

// surfaceRules are the rules of the surface, mesh-to-solid and retirement
// family (agent-docs/command-api-design.md section 12). None of these calls
// took an option, so each rule only moves ctx to Do. Stitch keeps its
// variadic operand slot, and Remove, which took no ctx, takes the context in
// scope at the call.
var surfaceRules = map[string]rule{
	"PatchEdges":    ruleModify("PatchEdges", 2, nil),
	"Stitch":        ruleStitch,
	"Unstitch":      ruleModify("Unstitch", 1, nil),
	"Trim":          ruleModify("Trim", 3, nil),
	"Extend":        ruleModify("Extend", 3, nil),
	"Split":         ruleModify("Split", 2, nil),
	"SolidFromMesh": ruleModify("SolidFromMesh", 2, nil),
	"Remove":        ruleRemove,
}

// ruleStitch: Stitch(ctx, a, b) -> Stitch(a, b).Do(ctx) and
// Stitch(ctx, bodies...) -> Stitch(bodies...).Do(ctx).
func ruleStitch(c *callSite) (string, string) {
	if len(c.args) < 2 {
		return "", "Stitch call without a context and a body"
	}
	return fmt.Sprintf("%s(%s%s).Do(%s)", qualify(c.pkg, "Stitch"), c.span(1, len(c.args)-1),
		map[bool]string{true: "...", false: ""}[c.ellipsis], c.text(c.args[0])), ""
}

// ruleRemove: Remove(b) -> Remove(b).Do(ctx). The old call took no ctx, so the
// rule needs one in scope: a ctx variable or parameter of the enclosing
// function, or the t.Context() of a testing parameter. Without one it
// reports, and the author supplies the context by hand.
func ruleRemove(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) != 1 {
		return "", "Remove call without exactly one body"
	}
	if c.enclosingCtx == "" {
		return "", "Remove call with no context in scope to pass to Do"
	}
	return fmt.Sprintf("%s(%s).Do(%s)", qualify(c.pkg, "Remove"), c.text(c.args[0]), c.enclosingCtx), ""
}
