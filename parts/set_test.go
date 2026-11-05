package parts_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSetRefusals(t *testing.T) {
	doc := decad.New()
	a, b := block(t, doc, 0, 0, 10, 10, 0, 5), block(t, doc, 20, 0, 30, 10, 0, 5)
	other := block(t, decad.New(), 0, 0, 10, 10, 0, 5)

	s := parts.NewSet(doc)
	mustAdd(t, s, parts.Part{Name: "a", Kind: parts.Manufactured, Body: a})

	for name, p := range map[string]parts.Part{
		"empty name":             {Kind: parts.Manufactured, Body: b},
		"repeated name":          {Name: "a", Kind: parts.Manufactured, Body: b},
		"unset kind":             {Name: "b", Body: b},
		"manufactured no body":   {Name: "b", Kind: parts.Manufactured},
		"body twice":             {Name: "b", Kind: parts.Manufactured, Body: a},
		"quantity with a body":   {Name: "b", Kind: parts.Manufactured, Body: b, Quantity: 2},
		"negative quantity":      {Name: "b", Kind: parts.Purchased, Quantity: -1},
		"negative body quantity": {Name: "b", Kind: parts.Manufactured, Body: b, Quantity: -1},
	} {
		require.ErrorIs(t, s.Add(p), decad.ErrDegenerate, name)
	}
	require.ErrorIs(t, s.Add(parts.Part{Name: "x", Kind: parts.Manufactured, Body: other}), decad.ErrForeignBody)
	require.Len(t, s.Parts(), 1)

	mustAdd(t, s, parts.Part{Name: "b", Kind: parts.Manufactured, Body: b},
		parts.Part{Name: "screw", Kind: parts.Purchased, Quantity: 4})
	_, err := s.Part("nope")
	require.ErrorIs(t, err, decad.ErrDegenerate)

	good := parts.Joint{Kind: parts.Contact, A: "a", B: "b"}
	for name, j := range map[string]parts.Joint{
		"unknown part":       {Kind: parts.Contact, A: "a", B: "zz"},
		"same part":          {Kind: parts.Contact, A: "a", B: "a"},
		"unset kind":         {A: "a", B: "b"},
		"keepout no gap":     {Kind: parts.Keepout, A: "a", B: "b"},
		"keepout moves":      {Kind: parts.Keepout, A: "a", B: "b", MinGap: units.Millimeters(1), Motion: parts.Turns},
		"contact moves":      {Kind: parts.Contact, A: "a", B: "b", Motion: parts.Slides},
		"face contact turns": {Kind: parts.FaceContact, A: "a", B: "b", Motion: parts.Turns},
		"slide no direction": {Kind: parts.FaceContact, A: "a", B: "b", Motion: parts.Slides},
		"axis on a contact":  {Kind: parts.Contact, A: "a", B: "b", Axis: &parts.Axis{Dir: r3.Vec{Z: 1}}},
		"axis no direction":  {Kind: parts.PinInBore, A: "a", B: "b", Axis: &parts.Axis{}},
		"bodyless geometric": {Kind: parts.PinInBore, A: "a", B: "screw"},
	} {
		require.ErrorIs(t, s.Join(j), decad.ErrDegenerate, name)
	}
	bad := good
	bad.Clearance = units.Degrees(1)
	require.ErrorIs(t, s.Join(bad), decad.ErrUnitKind)
	bad = good
	bad.MinGap = units.Millimeters(-1)
	require.ErrorIs(t, s.Join(bad), decad.ErrNegativeMagnitude)
	bad = good
	bad.Clearance = units.Millimeters(math.NaN())
	require.ErrorIs(t, s.Join(bad), decad.ErrNotFinite)
	bad = good
	bad.Direction = r3.Vec{X: math.Inf(1)}
	require.ErrorIs(t, s.Join(bad), decad.ErrNotFinite)

	// A tie may name a body-less part; an empty name becomes A~B, repeated
	// names are refused and Axis.Dir is normalised.
	mustJoin(t, s, parts.Joint{Kind: parts.Tie, A: "a", B: "screw"})
	require.Equal(t, "a~screw", s.Joints()[0].Name)
	require.ErrorIs(t, s.Join(parts.Joint{Kind: parts.Tie, A: "a", B: "screw"}), decad.ErrDegenerate)
	mustJoin(t, s, parts.Joint{Name: namePin, Kind: parts.PinInBore, A: "a", B: "b",
		Axis: &parts.Axis{Point: r3.Vec{X: 1}, Dir: r3.Vec{Z: 4}}})
	require.InDelta(t, 1.0, s.Joints()[1].Axis.Dir.Z, 1e-12)
}

func TestSetGroupAndRetirement(t *testing.T) {
	doc := decad.New()
	rig, err := doc.Root().NewChild("rig")
	require.NoError(t, err)
	inner, err := rig.NewChild("inner")
	require.NoError(t, err)
	a, b := block(t, doc, 0, 0, 10, 10, 0, 5), block(t, doc, 20, 0, 30, 10, 0, 5)
	require.NoError(t, doc.MoveBody(b, inner))

	s := parts.NewSet(doc)
	mustAdd(t, s, parts.Part{Name: "a", Kind: parts.Manufactured, Body: a},
		parts.Part{Name: "b", Kind: parts.Manufactured, Body: b},
		parts.Part{Name: "bolt", Kind: parts.Purchased, Quantity: 2})
	for name, want := range map[string]string{"a": "", "b": "rig/inner", "bolt": ""} {
		got, err := s.Group(name)
		require.NoError(t, err)
		require.Equal(t, want, got, name)
	}
	_, err = s.Group("missing")
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// Consuming a body in a later modelling step is reported by the next query.
	_, err = decad.Union(t.Context(), a, block(t, doc, 5, 0, 15, 10, 0, 5))
	require.NoError(t, err)
	_, err = s.Explode(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	_, err = s.Group("a")
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}
