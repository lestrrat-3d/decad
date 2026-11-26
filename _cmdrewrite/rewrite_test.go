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
		out, _, err := rewriteFile("x.go", []byte(in), families["booleans"], true)
		if err != nil || string(out) != "package decad\n\nfunc f() {\n\t_, _ = Cut(a, b).Do(ctx)\n}\n" {
			t.Fatalf("out %q err %v", out, err)
		}
	})
}
