package main

import (
	"fmt"
	"go/ast"
	"strings"
)

// compositeRules are the rules of the spec-composite family
// (agent-docs/command-api-design.md section 12). They match the pre-migration
// calls, Hole(ctx, b, decad.HoleSpec{...}) and the like, and split the spec
// literal into constructor arguments and method calls. A spec that is not a
// keyed composite literal of the spec type is reported, because the tool cannot
// see its fields.
var compositeRules = map[string]rule{
	"Hole":       ruleHole,
	"Boss":       ruleBoss,
	"Thread":     ruleThread,
	"Infill":     ruleInfill,
	"SheetMetal": ruleSheetMetal,
}

// specFields is the keyed fields of a spec literal, by field name.
type specFields map[string]ast.Expr

// specLiteral reads arg as a keyed composite literal of the decad type typeName
// and returns its fields. why is the reason it cannot.
func (c *callSite) specLiteral(arg ast.Expr, typeName string) (specFields, string) {
	lit, ok := arg.(*ast.CompositeLit)
	if !ok || lit.Type == nil || !c.isDecadType(lit.Type, typeName) {
		return nil, "the spec argument is not a literal of " + typeName + ", so its fields are not visible"
	}
	return c.fieldsOf(lit)
}

// fieldsOf reads the keyed fields of lit.
func (c *callSite) fieldsOf(lit *ast.CompositeLit) (specFields, string) {
	out := specFields{}
	for _, elt := range lit.Elts {
		kv, ok := elt.(*ast.KeyValueExpr)
		if !ok {
			return nil, "the spec literal has an unkeyed field"
		}
		key, ok := kv.Key.(*ast.Ident)
		if !ok {
			return nil, "the spec literal has a field with no name"
		}
		out[key.Name] = kv.Value
	}
	return out, ""
}

// isDecadType reports whether t names the decad type name in the qualifier form
// this call site uses.
func (c *callSite) isDecadType(t ast.Expr, name string) bool {
	switch v := t.(type) {
	case *ast.SelectorExpr:
		id, ok := v.X.(*ast.Ident)
		return ok && c.pkg != "" && id.Name == c.pkg && v.Sel.Name == name
	case *ast.Ident:
		return c.pkg == "" && v.Name == name
	}
	return false
}

// unsetValue reports whether e is the literal nil, the zero units.Value{} or the
// zero literal 0 or false: a value that left the field unset in the old spec.
func unsetValue(e ast.Expr) bool {
	switch v := e.(type) {
	case *ast.Ident:
		return v.Name == "nil" || v.Name == "false"
	case *ast.BasicLit:
		return v.Value == "0"
	case *ast.CompositeLit:
		return len(v.Elts) == 0 && v.Type != nil && strings.HasSuffix(typeText(v.Type), "Value")
	}
	return false
}

func typeText(t ast.Expr) string {
	switch v := t.(type) {
	case *ast.SelectorExpr:
		return typeText(v.X) + "." + v.Sel.Name
	case *ast.Ident:
		return v.Name
	}
	return ""
}

// present returns the field's expression when the old spec set it.
func (f specFields) present(name string) (ast.Expr, bool) {
	e, ok := f[name]
	if !ok || unsetValue(e) {
		return nil, false
	}
	return e, true
}

// zeroDefault reports a named field set by an expression that may be the zero
// value at run time. The old spec read zero as unset, and the new method validates
// the value it is given, so `.Draft(v)` with a zero `v` is refused where the old
// call took the default. Only a call (`units.Degrees(30)`) or a non-empty literal
// is known to be non-zero; the author decides what a variable meant.
func (f specFields) zeroDefault(names ...string) string {
	for _, n := range names {
		e, ok := f.present(n)
		if !ok {
			continue
		}
		switch v := e.(type) {
		case *ast.CallExpr:
			continue
		case *ast.CompositeLit:
			if len(v.Elts) > 0 {
				continue
			}
		}
		return "a " + n + " that is not a call or a literal and may be zero, which the old call took as unset"
	}
	return ""
}

// checkFields reports a field the rule does not know.
func (f specFields) checkFields(allowed ...string) string {
	known := map[string]struct{}{}
	for _, a := range allowed {
		known[a] = struct{}{}
	}
	for name := range f {
		if _, ok := known[name]; !ok {
			return "the spec literal sets " + name + ", which has no command rule"
		}
	}
	return ""
}

// required returns the texts of the named fields, in order, or the reason one is
// missing.
func (c *callSite) required(f specFields, names ...string) ([]string, string) {
	var out []string
	for _, n := range names {
		e, ok := f[n]
		if !ok {
			return nil, "the spec literal has no " + n
		}
		out = append(out, c.text(e))
	}
	return out, ""
}

// callArgs checks the shape of a spec composite call: a context, a first operand
// and the spec, with no spread.
func (c *callSite) callArgs(name string) string {
	if c.ellipsis || len(c.args) != 3 {
		return name + " call without exactly a context, an operand and a spec"
	}
	return ""
}

// literalOf reads e as a (possibly pointer to a) keyed literal of the decad type
// typeName.
func (c *callSite) literalOf(e ast.Expr, typeName string) (specFields, string) {
	if u, ok := e.(*ast.UnaryExpr); ok {
		e = u.X
	}
	lit, ok := e.(*ast.CompositeLit)
	if !ok || lit.Type == nil || !c.isDecadType(lit.Type, typeName) {
		return nil, "a " + typeName + " field is not a literal, so its fields are not visible"
	}
	return c.fieldsOf(lit)
}

// holeThread rewrites a &HoleThread{...} literal into NewHoleThread(...) and its value methods.
func (c *callSite) holeThread(e ast.Expr) (string, string) {
	f, why := c.literalOf(e, "HoleThread")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Pitch", "LeftHand", "Length"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Length"); why != "" {
		return "", why
	}
	pitch, why := c.required(f, "Pitch")
	if why != "" {
		return "", why
	}
	out := fmt.Sprintf("%s(%s)", qualify(c.pkg, "NewHoleThread"), pitch[0])
	if e, ok := f.present("LeftHand"); ok {
		if id, isID := e.(*ast.Ident); !isID || id.Name != "true" {
			return "", "a HoleThread LeftHand that is not the literal true"
		}
		out += ".LeftHand()"
	}
	if e, ok := f.present("Length"); ok {
		out += ".Length(" + c.text(e) + ")"
	}
	return out, ""
}

// holeOptions appends the Head, Thread and Draft methods of a hole or a boss bore. receiver is
// "" for a value builder chain.
func (c *callSite) holeOptions(f specFields, head, thread, draft bool) (string, string) {
	out := ""
	if e, ok := f.present("Head"); ok {
		if !head {
			return "", "a Head the new command has no method for"
		}
		out += ".Head(" + c.text(e) + ")"
	}
	if e, ok := f.present("Thread"); ok {
		if !thread {
			return "", "a Thread the new command has no method for"
		}
		t, why := c.holeThread(e)
		if why != "" {
			return "", why
		}
		out += ".Thread(" + t + ")"
	}
	if e, ok := f.present("Draft"); ok {
		if !draft {
			return "", "a Draft the new command has no method for"
		}
		out += ".Draft(" + c.text(e) + ")"
	}
	return out, ""
}

// ruleHole: Hole(ctx, b, decad.HoleSpec{Face: f, At: p, Diameter: d, Depth: x, Head: h, Thread: t, Draft: a}) ->
// Hole(b, f, p, d, x).Head(h).Thread(NewHoleThread(...)).Draft(a).Do(ctx). A Shape turns the call into
// HoleShaped(b, f, p, shape, x), which takes no diameter, head, thread or draft.
func ruleHole(c *callSite) (string, string) {
	if why := c.callArgs("Hole"); why != "" {
		return "", why
	}
	f, why := c.specLiteral(c.args[2], "HoleSpec")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Face", "At", "Diameter", "Depth", "Head", "Shape", "Thread", "Draft"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Draft"); why != "" {
		return "", why
	}
	ctx, body := c.text(c.args[0]), c.text(c.args[1])
	if shape, ok := f.present("Shape"); ok {
		if _, has := f.present("Diameter"); has {
			return "", "a shaped hole with a Diameter, which the old call refused"
		}
		for _, n := range []string{"Head", "Thread", "Draft"} {
			if _, has := f.present(n); has {
				return "", "a shaped hole with a " + n + ", which the old call refused"
			}
		}
		req, why := c.required(f, "Face", "At", "Depth")
		if why != "" {
			return "", why
		}
		return fmt.Sprintf("%s(%s, %s, %s, %s, %s).Do(%s)", qualify(c.pkg, "HoleShaped"),
			body, req[0], req[1], c.text(shape), req[2], ctx), ""
	}
	req, why := c.required(f, "Face", "At", "Diameter", "Depth")
	if why != "" {
		return "", why
	}
	opts, why := c.holeOptions(f, true, true, true)
	if why != "" {
		return "", why
	}
	return fmt.Sprintf("%s(%s, %s, %s, %s, %s)%s.Do(%s)", qualify(c.pkg, "Hole"),
		body, req[0], req[1], req[2], req[3], opts, ctx), ""
}

// bossBore rewrites a &BossBore{...} literal into NewBossBore(...) and its value methods.
func (c *callSite) bossBore(e ast.Expr) (string, string) {
	f, why := c.literalOf(e, "BossBore")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Diameter", "Depth", "Head", "Thread", "Draft"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Draft"); why != "" {
		return "", why
	}
	req, why := c.required(f, "Diameter", "Depth")
	if why != "" {
		return "", why
	}
	opts, why := c.holeOptions(f, true, true, true)
	if why != "" {
		return "", why
	}
	return fmt.Sprintf("%s(%s, %s)%s", qualify(c.pkg, "NewBossBore"), req[0], req[1], opts), ""
}

// bossRibs rewrites a &BossRibs{...} literal into NewBossRibs(...) and its Angle method.
func (c *callSite) bossRibs(e ast.Expr) (string, string) {
	f, why := c.literalOf(e, "BossRibs")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Count", "Thickness", "Length", "Height", "Angle"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Angle"); why != "" {
		return "", why
	}
	req, why := c.required(f, "Count", "Thickness", "Length", "Height")
	if why != "" {
		return "", why
	}
	out := fmt.Sprintf("%s(%s, %s, %s, %s)", qualify(c.pkg, "NewBossRibs"), req[0], req[1], req[2], req[3])
	if e, ok := f.present("Angle"); ok {
		out += ".Angle(" + c.text(e) + ")"
	}
	return out, ""
}

// ruleBoss: Boss(ctx, b, decad.BossSpec{Face: f, At: p, Diameter: d, Height: h, ...}) ->
// Boss(b, f, p, d, h)...Do(ctx). A Shape turns the call into BossShaped(b, f, p, shape, h). The old
// spec's Bore and Ribs literals become NewBossBore and NewBossRibs, and Tilt with TiltDirection becomes
// one Tilt call. A TiltDirection without a Tilt, which the old call refused, is reported.
func ruleBoss(c *callSite) (string, string) {
	if why := c.callArgs("Boss"); why != "" {
		return "", why
	}
	f, why := c.specLiteral(c.args[2], "BossSpec")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Face", "At", "Diameter", "Height", "Bore", "Draft", "RootFillet", "Shape", "Ribs",
		"Tilt", "TiltDirection"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Draft", "RootFillet", "Tilt", "TiltDirection"); why != "" {
		return "", why
	}
	ctx, body := c.text(c.args[0]), c.text(c.args[1])
	shape, shaped := f.present("Shape")
	var call string
	if shaped {
		if _, has := f.present("Diameter"); has {
			return "", "a shaped boss with a Diameter, which the old call refused"
		}
		req, why := c.required(f, "Face", "At", "Height")
		if why != "" {
			return "", why
		}
		call = fmt.Sprintf("%s(%s, %s, %s, %s, %s)", qualify(c.pkg, "BossShaped"), body, req[0], req[1], c.text(shape), req[2])
	} else {
		req, why := c.required(f, "Face", "At", "Diameter", "Height")
		if why != "" {
			return "", why
		}
		call = fmt.Sprintf("%s(%s, %s, %s, %s, %s)", qualify(c.pkg, "Boss"), body, req[0], req[1], req[2], req[3])
	}
	if e, ok := f.present("Bore"); ok {
		bore, why := c.bossBore(e)
		if why != "" {
			return "", why
		}
		call += ".Bore(" + bore + ")"
	}
	if e, ok := f.present("Draft"); ok {
		call += ".Draft(" + c.text(e) + ")"
	}
	if e, ok := f.present("RootFillet"); ok {
		if shaped {
			return "", "a shaped boss with a RootFillet, which the old call refused"
		}
		call += ".RootFillet(" + c.text(e) + ")"
	}
	if e, ok := f.present("Ribs"); ok {
		ribs, why := c.bossRibs(e)
		if why != "" {
			return "", why
		}
		call += ".Ribs(" + ribs + ")"
	}
	tilt, tilted := f.present("Tilt")
	direction, directed := f.present("TiltDirection")
	switch {
	case tilted && shaped:
		return "", "a shaped boss with a Tilt, which the old call refused"
	case directed && !tilted:
		return "", "a TiltDirection without a Tilt, which the old call refused"
	case tilted:
		dir := qualify("units", "Degrees") + "(0)"
		if directed {
			dir = c.text(direction)
		}
		call += ".Tilt(" + c.text(tilt) + ", " + dir + ")"
	}
	return call + ".Do(" + ctx + ")", ""
}

// threadForm rewrites a decad.ThreadForm{...} literal into its constructor.
func (c *callSite) threadForm(e ast.Expr) (string, string) {
	f, why := c.literalOf(e, "ThreadForm")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Angle", "Depth", "Flat", "FlankLead", "FlankTrail"); why != "" {
		return "", why
	}
	_, lead := f.present("FlankLead")
	_, trail := f.present("FlankTrail")
	_, angle := f.present("Angle")
	switch {
	case lead && trail && !angle:
		req, why := c.required(f, "FlankLead", "FlankTrail", "Depth", "Flat")
		if why != "" {
			return "", why
		}
		return fmt.Sprintf("%s(%s, %s, %s, %s)", qualify(c.pkg, "ButtressThreadForm"), req[0], req[1], req[2], req[3]), ""
	case angle && !lead && !trail:
		req, why := c.required(f, "Angle", "Depth", "Flat")
		if why != "" {
			return "", why
		}
		return fmt.Sprintf("%s(%s, %s, %s)", qualify(c.pkg, "TrapezoidThreadForm"), req[0], req[1], req[2]), ""
	}
	return "", "a ThreadForm that is neither a whole trapezoid nor a whole buttress, which the old call refused"
}

// ruleThread: Thread(ctx, b, decad.ThreadSpec{Face: f, Pitch: p, LeftHand: true, Length: l, Form: ..., LeadIn: d,
// LeadInAngle: a}) -> Thread(b, f, p).LeftHand().Length(l).Form(...).LeadInAngle(d, a).Do(ctx). A LeadInAngle
// without a LeadIn, which the old call refused, is reported.
func ruleThread(c *callSite) (string, string) {
	if why := c.callArgs("Thread"); why != "" {
		return "", why
	}
	f, why := c.specLiteral(c.args[2], "ThreadSpec")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Face", "Pitch", "LeftHand", "Length", "Form", "LeadIn", "LeadInAngle"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Length", "LeadIn", "LeadInAngle"); why != "" {
		return "", why
	}
	req, why := c.required(f, "Face", "Pitch")
	if why != "" {
		return "", why
	}
	call := fmt.Sprintf("%s(%s, %s, %s)", qualify(c.pkg, "Thread"), c.text(c.args[1]), req[0], req[1])
	if e, ok := f.present("LeftHand"); ok {
		if id, isID := e.(*ast.Ident); !isID || id.Name != "true" {
			return "", "a ThreadSpec LeftHand that is not the literal true"
		}
		call += ".LeftHand()"
	}
	if e, ok := f.present("Length"); ok {
		call += ".Length(" + c.text(e) + ")"
	}
	if e, ok := f.present("Form"); ok {
		form, why := c.threadForm(e)
		if why != "" {
			return "", why
		}
		call += ".Form(" + form + ")"
	}
	leadIn, hasLead := f.present("LeadIn")
	angle, hasAngle := f.present("LeadInAngle")
	switch {
	case hasAngle && !hasLead:
		return "", "a LeadInAngle without a LeadIn, which the old call refused"
	case hasAngle:
		call += ".LeadInAngle(" + c.text(leadIn) + ", " + c.text(angle) + ")"
	case hasLead:
		call += ".LeadIn(" + c.text(leadIn) + ")"
	}
	return call + ".Do(" + c.text(c.args[0]) + ")", ""
}

// ruleInfill: Infill(ctx, b, decad.InfillSpec{Kind: k, Cell: c, Level: l, Resolution: r, Skin: s}) ->
// Infill(b, k, c, l).Resolution(r).Skin(s).Do(ctx). A Kind the literal leaves out is the zero kind,
// GyroidInfill.
func ruleInfill(c *callSite) (string, string) {
	if why := c.callArgs("Infill"); why != "" {
		return "", why
	}
	f, why := c.specLiteral(c.args[2], "InfillSpec")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Kind", "Cell", "Level", "Resolution", "Skin"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("Skin"); why != "" {
		return "", why
	}
	req, why := c.required(f, "Cell", "Level")
	if why != "" {
		return "", why
	}
	kind := qualify(c.pkg, "GyroidInfill")
	if e, ok := f["Kind"]; ok {
		kind = c.text(e)
	}
	call := fmt.Sprintf("%s(%s, %s, %s, %s)", qualify(c.pkg, "Infill"), c.text(c.args[1]), kind, req[0], req[1])
	if e, ok := f.present("Resolution"); ok {
		call += ".Resolution(" + c.text(e) + ")"
	}
	if e, ok := f.present("Skin"); ok {
		call += ".Skin(" + c.text(e) + ")"
	}
	return call + ".Do(" + c.text(c.args[0]) + ")", ""
}

// ruleSheetMetal: SheetMetal(ctx, d, decad.SheetMetalSpec{Thickness: t, InnerRadius: r, Width: w, KFactor: k,
// Legs: l, Bends: b}) -> SheetMetal(d, t, r, w, l, b).KFactor(k).Do(ctx). Legs and Bends the literal leaves
// out are nil.
func ruleSheetMetal(c *callSite) (string, string) {
	if why := c.callArgs("SheetMetal"); why != "" {
		return "", why
	}
	f, why := c.specLiteral(c.args[2], "SheetMetalSpec")
	if why != "" {
		return "", why
	}
	if why := f.checkFields("Thickness", "InnerRadius", "Width", "KFactor", "Legs", "Bends"); why != "" {
		return "", why
	}
	if why := f.zeroDefault("KFactor"); why != "" {
		return "", why
	}
	req, why := c.required(f, "Thickness", "InnerRadius", "Width")
	if why != "" {
		return "", why
	}
	legs, bends := "nil", "nil"
	if e, ok := f["Legs"]; ok {
		legs = c.text(e)
	}
	if e, ok := f["Bends"]; ok {
		bends = c.text(e)
	}
	call := fmt.Sprintf("%s(%s, %s, %s, %s, %s, %s)", qualify(c.pkg, "SheetMetal"), c.text(c.args[1]),
		req[0], req[1], req[2], legs, bends)
	if e, ok := f.present("KFactor"); ok {
		call += ".KFactor(" + c.text(e) + ")"
	}
	return call + ".Do(" + c.text(c.args[0]) + ")", ""
}
