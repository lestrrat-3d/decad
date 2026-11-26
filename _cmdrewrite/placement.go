package main

import (
	"fmt"
	"go/ast"
)

// placementRules are the rules of the placement-and-patterns family
// (agent-docs/command-api-design.md section 12). They match the pre-migration
// names: Copy's three placement options, WithJoin, the WithPath... options and
// the WithArrange... options.
var placementRules = map[string]rule{
	"Transform":     ruleModify("Transform", 2, nil),
	"Copy":          ruleCopy,
	"Mirrored":      ruleMirrored,
	"MirroredCopy":  ruleModify("MirroredCopy", 2, nil),
	"ScaledAt":      ruleModify("ScaledAt", 3, nil),
	"QuarterTurn":   ruleModify("QuarterTurn", 4, nil),
	"Stretch":       ruleModify("Stretch", 5, nil),
	"PatternCopies": ruleModify("PatternCopies", 2, nil),
	"Patterned":     ruleModify("Patterned", 2, nil),
	"PathPattern":   rulePathPattern,
	"Arrange":       ruleArrange,
}

var ruleArrangeOptions = ruleModify("Arrange", 2, optionMethods{
	"WithArrangeTurns": {"Turns", 0}, "WithArrangeAngle": {"Angle", 1},
})

// ruleArrange rewrites Arrange through its option methods. The old call
// validated every WithArrangeAngle in order, so an invalid earlier one failed
// the call. Angle replaces its earlier call, so a repeated angle is reported,
// not rewritten into a call that now succeeds.
func ruleArrange(c *callSite) (string, string) {
	angles := 0
	if len(c.args) > 3 {
		for _, opt := range c.args[3:] {
			if name, _, _, _, ok := c.option(opt); ok && name == "WithArrangeAngle" {
				angles++
			}
		}
	}
	if angles > 1 {
		return "", "Arrange call with a repeated WithArrangeAngle, whose earlier value the old call validated"
	}
	return ruleArrangeOptions(c)
}

// ruleCopy: Copy(ctx, b, decad.WithCopyRotation(c, a, r)) ->
// Copy(b).Place(decad.CopyRotation{Center: c, Axis: a, Angle: r}).Do(ctx). Each
// placement option becomes the one Place call. The old call validated every
// placement option in order, so an invalid earlier option failed it. Place
// replaces its earlier call, so a call with more than one placement option is
// reported, not rewritten into a call that now succeeds.
func ruleCopy(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) < 2 {
		return "", "Copy call with a spread option list or too few arguments"
	}
	if len(c.args) > 3 {
		return "", "Copy call with more than one placement option, whose earlier options the old call validated"
	}
	call := fmt.Sprintf("%s(%s)", qualify(c.pkg, "Copy"), c.text(c.args[1]))
	for _, opt := range c.args[2:] {
		name, _, args, spread, ok := c.option(opt)
		if !ok || spread {
			return "", "Copy option " + c.text(opt) + " has no command method rule"
		}
		var variant string
		switch {
		case name == "WithCopyPlacement" && len(args) == 1:
			variant = fmt.Sprintf("%s{Transform: %s}", qualify(c.pkg, "CopyTransform"), c.text(args[0]))
		case name == "WithCopyRotation" && len(args) == 3:
			variant = fmt.Sprintf("%s{Center: %s, Axis: %s, Angle: %s}", qualify(c.pkg, "CopyRotation"),
				c.text(args[0]), c.text(args[1]), c.text(args[2]))
		case name == "WithCopyLiftTranslation" && len(args) == 3:
			variant = fmt.Sprintf("%s{Lift: %s, From: %s, To: %s}", qualify(c.pkg, "CopyLiftTranslation"),
				c.text(args[0]), c.text(args[1]), c.text(args[2]))
		default:
			return "", "Copy option " + c.text(opt) + " has no command method rule"
		}
		call += ".Place(" + variant + ")"
	}
	return call + ".Do(" + c.text(c.args[0]) + ")", ""
}

// ruleMirrored: Mirrored(ctx, b, plane) -> Mirrored(b, plane).Do(ctx), and
// Mirrored(ctx, b, plane, decad.WithJoin()) -> MirroredJoin(b, plane).Do(ctx).
func ruleMirrored(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) < 3 {
		return "", "Mirrored call with a spread option list or too few arguments"
	}
	name := "Mirrored"
	for _, opt := range c.args[3:] {
		if !c.isCall(opt, "WithJoin") {
			return "", "Mirrored option " + c.text(opt) + " is not WithJoin()"
		}
		name = "MirroredJoin"
	}
	return fmt.Sprintf("%s(%s).Do(%s)", qualify(c.pkg, name), c.span(1, 2), c.text(c.args[0])), ""
}

// rulePathPattern: PathPattern(ctx, b, path, decad.WithPathCount(n),
// decad.WithPathFollow()) -> PathPattern(b, path, decad.PathCount{Count: n}).Follow().Do(ctx).
// A pivot becomes FollowAbout. The old call refused a missing or repeated
// station option and a pivot without follow, so those are reported, not
// rewritten into a call that now succeeds.
func rulePathPattern(c *callSite) (string, string) {
	if c.ellipsis || len(c.args) < 3 {
		return "", "PathPattern call with a spread option list or too few arguments"
	}
	var stations string
	var follow bool
	var pivot ast.Expr
	for _, opt := range c.args[3:] {
		name, _, args, spread, ok := c.option(opt)
		if !ok || spread {
			return "", "PathPattern option " + c.text(opt) + " has no command method rule"
		}
		switch {
		case name == "WithPathCount" && len(args) == 1 && stations == "":
			stations = fmt.Sprintf("%s{Count: %s}", qualify(c.pkg, "PathCount"), c.text(args[0]))
		case name == "WithPathSpacing" && len(args) == 1 && stations == "":
			stations = fmt.Sprintf("%s{Spacing: %s}", qualify(c.pkg, "PathSpacing"), c.text(args[0]))
		case name == "WithPathFollow" && len(args) == 0:
			follow = true
		case name == "WithPathPivot" && len(args) == 1:
			pivot = args[0]
		default:
			return "", "PathPattern option " + c.text(opt) + " is repeated, conflicts or has no command method rule"
		}
	}
	if stations == "" {
		return "", "PathPattern call without exactly one of WithPathCount and WithPathSpacing"
	}
	if pivot != nil && !follow {
		return "", "PathPattern with WithPathPivot and no WithPathFollow, which the old call refused"
	}
	call := fmt.Sprintf("%s(%s, %s)", qualify(c.pkg, "PathPattern"), c.span(1, 2), stations)
	switch {
	case pivot != nil:
		call += ".FollowAbout(" + c.text(pivot) + ")"
	case follow:
		call += ".Follow()"
	}
	return call + ".Do(" + c.text(c.args[0]) + ")", ""
}
