package main

import "testing"

func TestBooleans(t *testing.T) {
	const head = "package p\n\nimport \"github.com/lestrrat-3d/decad\"\n\nfunc f() {\n"
	tests := []struct{ name, in, want string }{
		{"union", "_, _ = decad.Union(ctx, a, b, c)", "_, _ = decad.Union(a, b, c).Do(ctx)"},
		{"union spread", "_, _ = decad.Union(ctx, a, rest...)", "_, _ = decad.Union(append([]*decad.Body{a}, rest...)...).Do(ctx)"},
		{"cut", "_, _ = decad.Cut(ctx, a, b)", "_, _ = decad.Cut(a, b).Do(ctx)"},
		{"cut matching", "_, _ = decad.Cut(ctx, a, b, decad.WithMatchingFaces())", "_, _ = decad.Cut(a, b).MatchingFaces().Do(ctx)"},
		{"intersect", "_, _ = decad.Intersect(ctx, a, b)", "_, _ = decad.Intersect(a, b).Do(ctx)"},
		{"refine", "_, _ = decad.Refine(ctx, a, bound)", "_, _ = decad.Refine(a, bound).Do(ctx)"},
		{"union refined", "_, _ = decad.UnionRefined(ctx, bs, bound)", "_, _ = decad.Union(bs...).RefineTo(bound).Do(ctx)"},
		{"union refined literal", "_, _ = decad.UnionRefined(ctx, []*decad.Body{a, b, c}, bound)", "_, _ = decad.Union(a, b, c).RefineTo(bound).Do(ctx)"},
		{"nested", "_, _ = decad.Cut(ctx, mustBody(decad.Union(ctx, a, b)), c)", "_, _ = decad.Cut(mustBody(decad.Union(a, b).Do(ctx)), c).Do(ctx)"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			out, reports, err := rewriteFile("x.go", []byte(head+"\t"+tc.in+"\n}\n"), families["booleans"], false)
			if err != nil {
				t.Fatal(err)
			}
			if len(reports) != 0 {
				t.Fatalf("reports: %v", reports)
			}
			if want := head + "\t" + tc.want + "\n}\n"; string(out) != want {
				t.Fatalf("got\n%s\nwant\n%s", out, want)
			}
		})
	}
	t.Run("unknown option reported", func(t *testing.T) {
		in := head + "\t_, _ = decad.Cut(ctx, a, b, opts...)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["booleans"], false)
		if err != nil || len(reports) != 1 || string(out) != in {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("root bare call", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Cut(ctx, a, b)\n}\n"
		out, _, err := rewriteFile("x_test.go", []byte(in), families["booleans"], true)
		if err != nil || string(out) != "package decad\n\nfunc f() {\n\t_, _ = Cut(a, b).Do(ctx)\n}\n" {
			t.Fatalf("out %q err %v", out, err)
		}
	})
	t.Run("root production file reported", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Cut(ctx, a, b)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["booleans"], true)
		if err != nil || len(reports) != 1 || string(out) != in {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("multi-line arguments keep their breaks", func(t *testing.T) {
		in := head + "\t_, _ = decad.Cut(ctx,\n\t\tfirstBodyWithALongName(1, 2),\n\t\tsecondBodyWithALongName(3, 4),\n\t\tdecad.WithMatchingFaces())\n}\n"
		want := head + "\t_, _ = decad.Cut(firstBodyWithALongName(1, 2),\n\t\tsecondBodyWithALongName(3, 4)).MatchingFaces().Do(ctx)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["booleans"], false)
		if err != nil || len(reports) != 0 || string(out) != want {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("skipped directories", func(t *testing.T) {
		for name, want := range map[string]bool{
			".worktrees": true, ".git": true, ".tmp": true, "vendor": true, "testdata": true, "apitest": false,
		} {
			if skipDir(name) != want {
				t.Fatalf("skipDir(%q) != %v", name, want)
			}
		}
	})
}

func TestModify(t *testing.T) {
	const head = "package p\n\nimport \"github.com/lestrrat-3d/decad\"\n\nfunc f() {\n"
	tests := []struct{ name, in, want string }{
		{"fillet", "_, _ = decad.Fillet(ctx, b, sel, r)", "_, _ = decad.Fillet(b, sel, r).Do(ctx)"},
		{"fillet chain", "_, _ = decad.Fillet(ctx, b, sel, r, decad.WithTangentChain())",
			"_, _ = decad.Fillet(b, sel, r).TangentChain().Do(ctx)"},
		{"chamfer both options", "_, _ = decad.Chamfer(ctx, b, sel, d, decad.WithTangentChain(), decad.WithAsymmetricChamfer(ref, o))",
			"_, _ = decad.Chamfer(b, sel, d).TangentChain().Asymmetric(ref, o).Do(ctx)"},
		{"fillet chord", "_, _ = decad.FilletChord(ctx, b, sel, c)", "_, _ = decad.FilletChord(b, sel, c).Do(ctx)"},
		{"chamfer angle", "_, _ = decad.ChamferAngle(ctx, b, sel, d, ref, a)", "_, _ = decad.ChamferAngle(b, sel, d, ref, a).Do(ctx)"},
		{"shell", "_, _ = decad.Shell(ctx, b, sel, t)", "_, _ = decad.Shell(b, sel, t).Do(ctx)"},
		{"shell options", "_, _ = decad.Shell(ctx, b, sel, t, decad.WithShellSense(decad.Outward), decad.WithShellFaceThickness(n, u))",
			"_, _ = decad.Shell(b, sel, t).Sense(decad.Outward).FaceThickness(n, u).Do(ctx)"},
		{"shell closed", "_, _ = decad.Shell(ctx, b, nil, t, decad.WithNoOpenings())",
			"_, _ = decad.ShellClosed(b, t).Do(ctx)"},
		{"shell closed sense", "_, _ = decad.Shell(ctx, b, nil, t, decad.WithShellSense(decad.Centered), decad.WithNoOpenings())",
			"_, _ = decad.ShellClosed(b, t).Sense(decad.Centered).Do(ctx)"},
		{"draft", "_, _ = decad.Draft(ctx, b, sel, n, a)", "_, _ = decad.Draft(b, sel, n, a).Do(ctx)"},
		{"offset side", "_, _ = decad.Offset(ctx, b, d, decad.WithOffsetSide(decad.OffsetNegative))",
			"_, _ = decad.Offset(b, d).Side(decad.OffsetNegative).Do(ctx)"},
		{"thicken side", "_, _ = decad.Thicken(ctx, b, d, decad.WithThickenSide(decad.ThickenCentered))",
			"_, _ = decad.Thicken(b, d).Side(decad.ThickenCentered).Do(ctx)"},
		{"move face", "_, _ = decad.MoveFace(ctx, b, sel, d)", "_, _ = decad.MoveFace(b, sel, d).Do(ctx)"},
		{"cone from face", "_, _ = decad.ConeFromFace(ctx, b, sel, far)", "_, _ = decad.ConeFromFace(b, sel, far).Do(ctx)"},
		{"nested", "_, _ = decad.Thicken(ctx, mustBody(decad.Offset(ctx, b, d)), t)",
			"_, _ = decad.Thicken(mustBody(decad.Offset(b, d).Do(ctx)), t).Do(ctx)"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			out, reports, err := rewriteFile("x.go", []byte(head+"\t"+tc.in+"\n}\n"), families["modify"], false)
			if err != nil {
				t.Fatal(err)
			}
			if len(reports) != 0 {
				t.Fatalf("reports: %v", reports)
			}
			if want := head + "\t" + tc.want + "\n}\n"; string(out) != want {
				t.Fatalf("got\n%s\nwant\n%s", out, want)
			}
		})
	}
	for name, call := range map[string]string{
		"spread options":          "decad.Shell(ctx, b, sel, t, opts...)",
		"option variable":         "decad.Fillet(ctx, b, sel, r, opt)",
		"closed with a selector":  "decad.Shell(ctx, b, sel, t, decad.WithNoOpenings())",
		"option with wrong arity": "decad.Chamfer(ctx, b, sel, d, decad.WithAsymmetricChamfer(ref))",
		"draft option":            "decad.Draft(ctx, b, sel, n, a, opt)",
	} {
		t.Run("reported "+name, func(t *testing.T) {
			in := head + "\t_, _ = " + call + "\n}\n"
			out, reports, err := rewriteFile("x.go", []byte(in), families["modify"], false)
			if err != nil || len(reports) != 1 || string(out) != in {
				t.Fatalf("out %q reports %v err %v", out, reports, err)
			}
		})
	}
	t.Run("multi-line option arguments keep their breaks", func(t *testing.T) {
		in := head + "\t_, _ = decad.Chamfer(ctx, b,\n\t\tsel,\n\t\td,\n\t\tdecad.WithAsymmetricChamfer(\n\t\t\tref,\n\t\t\to))\n}\n"
		want := head + "\t_, _ = decad.Chamfer(b,\n\t\tsel,\n\t\td).Asymmetric(ref,\n\t\to).Do(ctx)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["modify"], false)
		if err != nil || len(reports) != 0 || string(out) != want {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("root bare call", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Shell(ctx, b, nil, t, WithNoOpenings())\n}\n"
		out, _, err := rewriteFile("x_test.go", []byte(in), families["modify"], true)
		if err != nil || string(out) != "package decad\n\nfunc f() {\n\t_, _ = ShellClosed(b, t).Do(ctx)\n}\n" {
			t.Fatalf("out %q err %v", out, err)
		}
	})
}

// The new method names are not old-API input, so the tool leaves a call that
// already uses them alone.
func TestModifyLeavesNewNamesAlone(t *testing.T) {
	const head = "package p\n\nimport \"github.com/lestrrat-3d/decad\"\n\nfunc f() {\n"
	in := head + "\t_, _ = decad.Fillet(ctx, b, sel, r, decad.TangentChain())\n}\n"
	out, reports, err := rewriteFile("x.go", []byte(in), families["modify"], false)
	if err != nil || len(reports) != 1 || string(out) != in {
		t.Fatalf("out %q reports %v err %v", out, reports, err)
	}
}

func TestFeatures(t *testing.T) {
	const head = "package p\n\nimport \"github.com/lestrrat-3d/decad\"\n\nfunc f() {\n"
	tests := []struct{ name, in, want string }{
		{"extrude", "_, _ = decad.Extrude(ctx, d, s, p, e)", "_, _ = decad.Extrude(d, s, p, e).Do(ctx)"},
		{"extrude options", "_, _ = decad.Extrude(ctx, d, s, p, e, decad.WithTaper(a), decad.WithSurfaceResult())",
			"_, _ = decad.Extrude(d, s, p, e).Taper(a).SurfaceResult().Do(ctx)"},
		{"extrude explicit type argument", "_, _ = decad.Extrude[*sketch.Chain](ctx, d, s, c, e)",
			"_, _ = decad.Extrude(d, s, c, e).Do(ctx)"},
		{"revolve", "_, _ = decad.Revolve(ctx, d, s, p, ax, a, decad.WithSurfaceResult())",
			"_, _ = decad.Revolve(d, s, p, ax, a).SurfaceResult().Do(ctx)"},
		{"sweep options", "_, _ = decad.Sweep(ctx, d, s, p, path, decad.WithSweepTwist(a), decad.WithMitredJoins(), decad.WithSectionScale(x, y), decad.WithSurfaceResult())",
			"_, _ = decad.Sweep(d, s, p, path).Twist(a).MitredJoins().SectionScale(x, y).SurfaceResult().Do(ctx)"},
		{"sweep empty section scale", "_, _ = decad.Sweep(ctx, d, s, p, path, decad.WithSectionScale())",
			"_, _ = decad.Sweep(d, s, p, path).SectionScale().Do(ctx)"},
		{"sweep spread section scale", "_, _ = decad.Sweep(ctx, d, s, p, path, decad.WithSectionScale(factors...))",
			"_, _ = decad.Sweep(d, s, p, path).SectionScale(factors...).Do(ctx)"},
		{"loft spread alignment", "_, _ = decad.Loft(ctx, d, s0, p0, s1, p1, decad.WithLoftAlignment(offs...))",
			"_, _ = decad.Loft(d, s0, p0, s1, p1).Alignment(offs...).Do(ctx)"},
		{"coil", "_, _ = decad.Coil(ctx, d, s, p, ax, pitch, turns, decad.WithLeftHand())",
			"_, _ = decad.Coil(d, s, p, ax, pitch, turns).LeftHand().Do(ctx)"},
		{"loft options", "_, _ = decad.Loft(ctx, d, s0, p0, s1, p1, decad.WithLoftAlignment(1, 2), decad.WithLoftSplits(a, b), decad.WithSurfaceResult())",
			"_, _ = decad.Loft(d, s0, p0, s1, p1).Alignment(1, 2).Splits(a, b).SurfaceResult().Do(ctx)"},
		{"loft chain", "_, _ = decad.LoftChain(ctx, d, s0, c0, s1, c1)", "_, _ = decad.LoftChain(d, s0, c0, s1, c1).Do(ctx)"},
		{"loft from point", "_, _ = decad.LoftFromPoint(ctx, d, apex, s, p)", "_, _ = decad.LoftFromPoint(d, apex, s, p).Do(ctx)"},
		{"patch profile", "_, _ = decad.PatchProfile(ctx, d, s, p)", "_, _ = decad.PatchProfile(d, s, p).Do(ctx)"},
		{"loft sections", "_, _ = decad.LoftSections(ctx, d, a, b, c)", "_, _ = decad.LoftSections(d, a, b, c).Do(ctx)"},
		{"loft sections spread", "_, _ = decad.LoftSections(ctx, d, secs...)", "_, _ = decad.LoftSections(d, secs...).Do(ctx)"},
		{"loft sections circular", "_, _ = decad.LoftSectionsWithCircularCorrespondence(ctx, d, corr, a, b, c)",
			"_, _ = decad.LoftSections(d, a, b, c).CircularCorrespondence(corr).Do(ctx)"},
		{"loft sections circular spread", "_, _ = decad.LoftSectionsWithCircularCorrespondence(ctx, d, corr, secs...)",
			"_, _ = decad.LoftSections(d, secs...).CircularCorrespondence(corr).Do(ctx)"},
		{"loft faces", "_, _ = decad.LoftFaces(ctx, faces)", "_, _ = decad.LoftFaces(faces).Do(ctx)"},
		{"loft faces cone", "_, _ = decad.LoftFaces(ctx, faces, decad.WithConeTangentEnd(cone, radial))",
			"_, _ = decad.LoftFacesToCone(faces, cone, radial).Do(ctx)"},
		{"emboss", "_, _ = decad.Emboss(ctx, b, s, ps, face, decad.EmbossRaised, h, decad.WithEmbossRimFillet(r), decad.WithEmbossDraft(a))",
			"_, _ = decad.Emboss(b, s, ps, face, decad.EmbossRaised, h).RimFillet(r).Draft(a).Do(ctx)"},
		{"web path", "_, _ = decad.WebPath(ctx, b, s, path, t, e, decad.WithRibDraft(a), decad.WithRibOverrun(o))",
			"_, _ = decad.WebPath(b, s, path, t, e).Draft(a).Overrun(o).Do(ctx)"},
		{"cut extrude", "_, _ = decad.CutExtrude(ctx, b, s, ps, e, decad.WithTaper(a))", "_, _ = decad.CutExtrude(b, s, ps, e).Taper(a).Do(ctx)"},
		{"cut revolve", "_, _ = decad.CutRevolve(ctx, b, s, ps, ax, a)", "_, _ = decad.CutRevolve(b, s, ps, ax, a).Do(ctx)"},
		{"nested", "_, _ = decad.Cut(ctx, mustBody(decad.Extrude(ctx, d, s, p, e)), c)",
			"_, _ = decad.Cut(ctx, mustBody(decad.Extrude(d, s, p, e).Do(ctx)), c)"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			out, reports, err := rewriteFile("x.go", []byte(head+"\t"+tc.in+"\n}\n"), families["features"], false)
			if err != nil {
				t.Fatal(err)
			}
			if len(reports) != 0 {
				t.Fatalf("reports: %v", reports)
			}
			if want := head + "\t" + tc.want + "\n}\n"; string(out) != want {
				t.Fatalf("got\n%s\nwant\n%s", out, want)
			}
		})
	}
	for name, call := range map[string]string{
		"spread options":                       "decad.Extrude(ctx, d, s, p, e, opts...)",
		"option variable":                      "decad.Revolve(ctx, d, s, p, ax, a, opt)",
		"cut extrude surface":                  "decad.CutExtrude(ctx, b, s, ps, e, decad.WithSurfaceResult())",
		"option with wrong arity":              "decad.Extrude(ctx, d, s, p, e, decad.WithTaper())",
		"spread tail on a fixed option":        "decad.Extrude(ctx, d, s, p, e, decad.WithTaper(xs...))",
		"spread tail on a no-argument option":  "decad.Revolve(ctx, d, s, p, ax, a, decad.WithSurfaceResult(xs...))",
		"spread tail on a two-argument option": "decad.Loft(ctx, d, s0, p0, s1, p1, decad.WithLoftSplits(xs...))",
		"spread tail on the cone option":       "decad.LoftFaces(ctx, faces, decad.WithConeTangentEnd(xs...))",
		"loft faces option":                    "decad.LoftFaces(ctx, faces, opt)",
		"loft faces spread options":            "decad.LoftFaces(ctx, faces, opts...)",
	} {
		t.Run("reported "+name, func(t *testing.T) {
			in := head + "\t_, _ = " + call + "\n}\n"
			out, reports, err := rewriteFile("x.go", []byte(in), families["features"], false)
			if err != nil || len(reports) != 1 || string(out) != in {
				t.Fatalf("out %q reports %v err %v", out, reports, err)
			}
		})
	}
	t.Run("root bare call", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Extrude(ctx, d, s, p, e, WithTaper(a))\n}\n"
		out, _, err := rewriteFile("x_test.go", []byte(in), families["features"], true)
		if err != nil || string(out) != "package decad\n\nfunc f() {\n\t_, _ = Extrude(d, s, p, e).Taper(a).Do(ctx)\n}\n" {
			t.Fatalf("out %q err %v", out, err)
		}
	})
	t.Run("root production file reported", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Extrude(ctx, d, s, p, e)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["features"], true)
		if err != nil || len(reports) != 1 || string(out) != in {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("new names are not old-API input", func(t *testing.T) {
		in := head + "\t_, _ = decad.Extrude(ctx, d, s, p, e, decad.Taper(a))\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["features"], false)
		if err != nil || len(reports) != 1 || string(out) != in {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
}

func TestPlacement(t *testing.T) {
	const head = "package p\n\nimport \"github.com/lestrrat-3d/decad\"\n\nfunc f() {\n"
	tests := []struct{ name, in, want string }{
		{"transform", "_, _ = decad.Transform(ctx, b, t)", "_, _ = decad.Transform(b, t).Do(ctx)"},
		{"copy", "_, _ = decad.Copy(ctx, b)", "_, _ = decad.Copy(b).Do(ctx)"},
		{"copy placement", "_, _ = decad.Copy(ctx, b, decad.WithCopyPlacement(t))",
			"_, _ = decad.Copy(b).Place(decad.CopyTransform{Transform: t}).Do(ctx)"},
		{"copy rotation", "_, _ = decad.Copy(ctx, b, decad.WithCopyRotation(c, ax, a))",
			"_, _ = decad.Copy(b).Place(decad.CopyRotation{Center: c, Axis: ax, Angle: a}).Do(ctx)"},
		{"copy lift translation", "_, _ = decad.Copy(ctx, b, decad.WithCopyLiftTranslation(l, from, to))",
			"_, _ = decad.Copy(b).Place(decad.CopyLiftTranslation{Lift: l, From: from, To: to}).Do(ctx)"},
		{"mirrored", "_, _ = decad.Mirrored(ctx, b, plane)", "_, _ = decad.Mirrored(b, plane).Do(ctx)"},
		{"mirrored join", "_, _ = decad.Mirrored(ctx, b, plane, decad.WithJoin())", "_, _ = decad.MirroredJoin(b, plane).Do(ctx)"},
		{"mirrored copy", "_, _ = decad.MirroredCopy(ctx, b, plane)", "_, _ = decad.MirroredCopy(b, plane).Do(ctx)"},
		{"scaled at", "_, _ = decad.ScaledAt(ctx, b, anchor, f)", "_, _ = decad.ScaledAt(b, anchor, f).Do(ctx)"},
		{"quarter turn", "_, _ = decad.QuarterTurn(ctx, b, ax, about, 1)", "_, _ = decad.QuarterTurn(b, ax, about, 1).Do(ctx)"},
		{"stretch", "_, _ = decad.Stretch(ctx, b, anchor, x, y, z)", "_, _ = decad.Stretch(b, anchor, x, y, z).Do(ctx)"},
		{"pattern copies", "_, _ = decad.PatternCopies(ctx, b, spec)", "_, _ = decad.PatternCopies(b, spec).Do(ctx)"},
		{"patterned", "_, _ = decad.Patterned(ctx, b, spec)", "_, _ = decad.Patterned(b, spec).Do(ctx)"},
		{"path pattern count", "_, _ = decad.PathPattern(ctx, b, path, decad.WithPathCount(4))",
			"_, _ = decad.PathPattern(b, path, decad.PathCount{Count: 4}).Do(ctx)"},
		{"path pattern spacing follow", "_, _ = decad.PathPattern(ctx, b, path, decad.WithPathFollow(), decad.WithPathSpacing(s))",
			"_, _ = decad.PathPattern(b, path, decad.PathSpacing{Spacing: s}).Follow().Do(ctx)"},
		{"path pattern pivot", "_, _ = decad.PathPattern(ctx, b, path, decad.WithPathCount(3), decad.WithPathPivot(p), decad.WithPathFollow())",
			"_, _ = decad.PathPattern(b, path, decad.PathCount{Count: 3}).FollowAbout(p).Do(ctx)"},
		{"arrange", "_, _ = decad.Arrange(ctx, bodies, bed)", "_, _ = decad.Arrange(bodies, bed).Do(ctx)"},
		{"arrange options", "_, _ = decad.Arrange(ctx, bodies, bed, decad.WithArrangeTurns(), decad.WithArrangeAngle(a))",
			"_, _ = decad.Arrange(bodies, bed).Turns().Angle(a).Do(ctx)"},
		{"nested", "_, _ = decad.Transform(ctx, mustBody(decad.Copy(ctx, b)), t)",
			"_, _ = decad.Transform(mustBody(decad.Copy(b).Do(ctx)), t).Do(ctx)"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			out, reports, err := rewriteFile("x.go", []byte(head+"\t"+tc.in+"\n}\n"), families["placement"], false)
			if err != nil {
				t.Fatal(err)
			}
			if len(reports) != 0 {
				t.Fatalf("reports: %v", reports)
			}
			if want := head + "\t" + tc.want + "\n}\n"; string(out) != want {
				t.Fatalf("got\n%s\nwant\n%s", out, want)
			}
		})
	}
	for name, call := range map[string]string{
		"copy spread options":            "decad.Copy(ctx, b, opts...)",
		"copy option variable":           "decad.Copy(ctx, b, opt)",
		"copy placement wrong arity":     "decad.Copy(ctx, b, decad.WithCopyRotation(c, a))",
		"mirrored option variable":       "decad.Mirrored(ctx, b, plane, opt)",
		"path pattern no stations":       "decad.PathPattern(ctx, b, path, decad.WithPathFollow())",
		"path pattern both stations":     "decad.PathPattern(ctx, b, path, decad.WithPathCount(3), decad.WithPathSpacing(s))",
		"path pattern repeated count":    "decad.PathPattern(ctx, b, path, decad.WithPathCount(3), decad.WithPathCount(4))",
		"path pattern pivot no follow":   "decad.PathPattern(ctx, b, path, decad.WithPathCount(3), decad.WithPathPivot(p))",
		"path pattern spread options":    "decad.PathPattern(ctx, b, path, opts...)",
		"copy repeated placement":        "decad.Copy(ctx, b, decad.WithCopyPlacement(t), decad.WithCopyRotation(c, ax, a))",
		"copy repeated invalid rotation": "decad.Copy(ctx, b, decad.WithCopyRotation(c, zero, a), decad.WithCopyPlacement(t))",
		"arrange repeated angle":         "decad.Arrange(ctx, bodies, bed, decad.WithArrangeAngle(a), decad.WithArrangeAngle(b))",
		"arrange spread options":         "decad.Arrange(ctx, bodies, bed, opts...)",
		"arrange spread angle":           "decad.Arrange(ctx, bodies, bed, decad.WithArrangeAngle(xs...))",
		"transform with a stray option":  "decad.Transform(ctx, b, t, opt)",
	} {
		t.Run("reported "+name, func(t *testing.T) {
			in := head + "\t_, _ = " + call + "\n}\n"
			out, reports, err := rewriteFile("x.go", []byte(in), families["placement"], false)
			if err != nil || len(reports) != 1 || string(out) != in {
				t.Fatalf("out %q reports %v err %v", out, reports, err)
			}
		})
	}
	t.Run("root bare call", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Copy(ctx, b, WithCopyPlacement(t))\n}\n"
		out, _, err := rewriteFile("x_test.go", []byte(in), families["placement"], true)
		want := "package decad\n\nfunc f() {\n\t_, _ = Copy(b).Place(CopyTransform{Transform: t}).Do(ctx)\n}\n"
		if err != nil || string(out) != want {
			t.Fatalf("out %q err %v", out, err)
		}
	})
	t.Run("root production file reported", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Copy(ctx, b)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["placement"], true)
		if err != nil || len(reports) != 1 || string(out) != in {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("new names are not old-API input", func(t *testing.T) {
		in := head + "\t_, _ = decad.Copy(ctx, b, decad.CopyTransform{Transform: t})\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["placement"], false)
		if err != nil || len(reports) != 1 || string(out) != in {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
}
