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
		{"fillet chain", "_, _ = decad.Fillet(ctx, b, sel, r, decad.TangentChain())",
			"_, _ = decad.Fillet(b, sel, r).TangentChain().Do(ctx)"},
		{"chamfer both options", "_, _ = decad.Chamfer(ctx, b, sel, d, decad.TangentChain(), decad.Asymmetric(ref, o))",
			"_, _ = decad.Chamfer(b, sel, d).TangentChain().Asymmetric(ref, o).Do(ctx)"},
		{"fillet chord", "_, _ = decad.FilletChord(ctx, b, sel, c)", "_, _ = decad.FilletChord(b, sel, c).Do(ctx)"},
		{"chamfer angle", "_, _ = decad.ChamferAngle(ctx, b, sel, d, ref, a)", "_, _ = decad.ChamferAngle(b, sel, d, ref, a).Do(ctx)"},
		{"shell", "_, _ = decad.Shell(ctx, b, sel, t)", "_, _ = decad.Shell(b, sel, t).Do(ctx)"},
		{"shell options", "_, _ = decad.Shell(ctx, b, sel, t, decad.Sense(decad.Outward), decad.FaceThickness(n, u))",
			"_, _ = decad.Shell(b, sel, t).Sense(decad.Outward).FaceThickness(n, u).Do(ctx)"},
		{"shell closed", "_, _ = decad.Shell(ctx, b, nil, t, decad.ShellClosed())",
			"_, _ = decad.ShellClosed(b, t).Do(ctx)"},
		{"shell closed sense", "_, _ = decad.Shell(ctx, b, nil, t, decad.Sense(decad.Centered), decad.ShellClosed())",
			"_, _ = decad.ShellClosed(b, t).Sense(decad.Centered).Do(ctx)"},
		{"draft", "_, _ = decad.Draft(ctx, b, sel, n, a)", "_, _ = decad.Draft(b, sel, n, a).Do(ctx)"},
		{"offset side", "_, _ = decad.Offset(ctx, b, d, decad.Side(decad.OffsetNegative))",
			"_, _ = decad.Offset(b, d).Side(decad.OffsetNegative).Do(ctx)"},
		{"thicken side", "_, _ = decad.Thicken(ctx, b, d, decad.Side(decad.ThickenCentered))",
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
		"closed with a selector":  "decad.Shell(ctx, b, sel, t, decad.ShellClosed())",
		"option with wrong arity": "decad.Chamfer(ctx, b, sel, d, decad.Asymmetric(ref))",
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
		in := head + "\t_, _ = decad.Chamfer(ctx, b,\n\t\tsel,\n\t\td,\n\t\tdecad.Asymmetric(\n\t\t\tref,\n\t\t\to))\n}\n"
		want := head + "\t_, _ = decad.Chamfer(b,\n\t\tsel,\n\t\td).Asymmetric(ref,\n\t\to).Do(ctx)\n}\n"
		out, reports, err := rewriteFile("x.go", []byte(in), families["modify"], false)
		if err != nil || len(reports) != 0 || string(out) != want {
			t.Fatalf("out %q reports %v err %v", out, reports, err)
		}
	})
	t.Run("root bare call", func(t *testing.T) {
		in := "package decad\n\nfunc f() {\n\t_, _ = Shell(ctx, b, nil, t, ShellClosed())\n}\n"
		out, _, err := rewriteFile("x_test.go", []byte(in), families["modify"], true)
		if err != nil || string(out) != "package decad\n\nfunc f() {\n\t_, _ = ShellClosed(b, t).Do(ctx)\n}\n" {
			t.Fatalf("out %q err %v", out, err)
		}
	})
}
