package meshbool

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type pairDescriptor struct {
	Facets       []pairFacet `json:"Facets"`
	MaxRim       float64     `json:"MaxRim"`
	ErrorType    string      `json:"ErrorType"`
	ContactKind  string      `json:"ContactKind"`
	ContactCount int         `json:"ContactCount"`
}
type pairFacet struct {
	Points [3]string  `json:"Points"`
	Keys   [3]string  `json:"Keys"`
	Src    int        `json:"Src"`
	Beta   [3]float64 `json:"Beta"`
}

func TestPairCutsBaseline(t *testing.T) {
	identity := func(v r3.Vec) r3.Vec { return v }
	cases := []struct {
		name         string
		at           r3.Vec
		rot          func(r3.Vec) r3.Vec
		held, cancel bool
	}{
		{"crossing pair", r3.Vec{X: .5, Y: .5, Z: .5}, identity, false, false},
		{"coplanar", r3.Vec{X: 2}, identity, false, false},
		{"edge", r3.Vec{X: 3, Z: .3}, func(v r3.Vec) r3.Vec { return r3.Vec{X: v.X - v.Y, Y: v.X + v.Y, Z: v.Z} }, false, false},
		{"point", r3.Vec{X: 2, Y: 2, Z: 2}, identity, false, false},
		{"cancel", r3.Vec{X: .5, Y: .5, Z: .5}, identity, false, true},
		{"held", r3.Vec{X: .5, Y: .5, Z: .5}, identity, true, false},
	}
	out := make(map[string]pairDescriptor)
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			a := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{})
			b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, tc.rot, tc.at)
			a.SetVertexBounds([]float64{.01, .01, .01, .01, .01, .01, .01, .01})
			b.SetVertexBounds([]float64{.02, .02, .02, .02, .02, .02, .02, .02})
			if tc.held {
				a.Gate = .001
			}
			ctx := t.Context()
			if tc.cancel {
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			}
			kept, rim, err := meshBoolean(ctx, OpUnion, a, b, NewContactMemo(a, b), newKeyMemo)
			d := pairDescriptor{MaxRim: rim}
			if err != nil {
				d.ErrorType = fmt.Sprintf("%T", err)
				if touch, ok := errors.AsType[*TouchContactError](err); ok {
					d.ContactKind = touch.What
					d.ContactCount = len(touch.Contacts)
				}
				if _, ok := errors.AsType[*CoarseHeldContactError](err); ok {
					d.ErrorType += "/CoarseHeldContactError"
				}
			}
			for _, f := range kept {
				x := pairFacet{Keys: f.Keys, Src: f.Src, Beta: f.Beta}
				for k := range 3 {
					x.Points[k] = base64.StdEncoding.EncodeToString([]byte(f.V[k].Key()))
					x.Keys[k] = base64.StdEncoding.EncodeToString([]byte(f.Keys[k]))
					require.Equal(t, x.Points[k], x.Keys[k])
				}
				d.Facets = append(d.Facets, x)
			}
			out[tc.name] = d
		})
	}
	data, err := os.ReadFile("testdata/paircuts_baseline.json")
	require.NoError(t, err)
	var want map[string]pairDescriptor
	require.NoError(t, json.Unmarshal(data, &want))
	for name, got := range out {
		expected := want[name]
		require.Equal(t, expected.ErrorType, got.ErrorType, "%s error type", name)
		require.Equal(t, expected.ContactKind, got.ContactKind, "%s contact kind", name)
		require.Equal(t, expected.ContactCount, got.ContactCount, "%s contact count", name)
		require.Equal(t, expected.MaxRim, got.MaxRim, "%s rim", name)
		require.Len(t, got.Facets, len(expected.Facets), "%s facets", name)
		for i, facet := range got.Facets {
			require.Equal(t, expected.Facets[i].Src, facet.Src, "%s facet %d source", name, i)
			for k := range 3 {
				require.Equal(t, expected.Facets[i].Points[k], facet.Points[k], "%s facet %d corner %d point", name, i, k)
				require.Equal(t, expected.Facets[i].Keys[k], facet.Keys[k], "%s facet %d corner %d key", name, i, k)
				require.Equal(t, expected.Facets[i].Beta[k], facet.Beta[k], "%s facet %d corner %d bound", name, i, k)
			}
		}
	}
}

func TestPairCutsCoplanarDiagnosticCap(t *testing.T) {
	identity := func(v r3.Vec) r3.Vec { return v }
	original := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{})
	tris := make([][3]int, 0, len(original.Tris)*10)
	for range 10 {
		tris = append(tris, original.Tris...)
	}
	src := make([]int, len(tris))
	a, err := PrepContext(t.Context(), original.Verts, tris, src)
	require.NoError(t, err)
	a.SetVertexBounds(make([]float64, len(a.Verts)))
	b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{})
	_, _, err = meshBoolean(t.Context(), OpUnion, a, b, NewContactMemo(a, b), newKeyMemo)
	var touch *TouchContactError
	require.ErrorAs(t, err, &touch)
	require.Equal(t, touchCoplanar, touch.What)
	require.Len(t, touch.Contacts, 64)
}
