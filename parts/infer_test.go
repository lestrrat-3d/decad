package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func inferredNames(ij []parts.InferredJoint) []string {
	var out []string
	for _, j := range ij {
		out = append(out, j.Joint.Name)
	}
	return out
}

func TestInfer(t *testing.T) {
	// A lap with two pins through both plates: each pin sits in each plate's bore, 0.2 mm
	// all round, and nothing else is a pin joint.
	s := lapSet(t)
	got, err := s.Infer(t.Context())
	require.NoError(t, err)
	require.ElementsMatch(t, []string{jointPin1Base, "pin1~top", "pin2~base", "pin2~top"}, inferredNames(got))
	for _, ij := range got {
		require.Equal(t, parts.PinInBore, ij.Joint.Kind)
		require.Equal(t, parts.Fixed, ij.Joint.Motion)
		require.InDelta(t, 0.2, ij.Measured.Value.Base(), 1e-9)
		require.InDelta(t, 0.2, ij.Joint.Clearance.Base(), 1e-9)
		require.InDelta(t, 1.0, ij.Joint.Axis.Dir.Z*ij.Joint.Axis.Dir.Z, 1e-9)
		require.NotEmpty(t, ij.FacesA)
		require.NotEmpty(t, ij.FacesB)
	}
	// The pin's axis passes through the pin's own centre line.
	for _, ij := range got {
		want := 5.0
		if ij.Joint.A == namePin2 {
			want = 15
		}
		require.InDelta(t, want, ij.Joint.Axis.Point.X, 1e-9, ij.Joint.Name)
	}
	// It added nothing to the Set.
	require.Len(t, s.Joints(), 5)
}

func TestInferSlotAndRefusals(t *testing.T) {
	doc := decad.New()
	blk, err := decad.Cut(t.Context(), block(t, doc, 0, 0, 30, 20, 0, 10), block(t, doc, 12.8, 5, 17.2, 15, -1, 12))
	require.NoError(t, err)
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: blk},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 15, 2, -2, 15)},
		parts.Part{Name: "far", Kind: parts.Manufactured, Body: cylinder(t, doc, 80, 2, 0, 10)},
		parts.Part{Name: nameStrap, Kind: parts.Purchased, Quantity: 1})
	got, err := s.Infer(t.Context())
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.Equal(t, parts.PinInSlot, got[0].Joint.Kind)
	require.Equal(t, namePin, got[0].Joint.A)
	require.Equal(t, nameBlock, got[0].Joint.B)
	require.InDelta(t, 0.4, got[0].Measured.Value.Base(), 1e-9)
	require.Len(t, got[0].FacesB, 2)

	// A tighter MaxFit refuses the 0.4 mm slot.
	got, err = s.Infer(t.Context(), parts.WithMaxFit(units.Millimeters(0.3)))
	require.NoError(t, err)
	require.Empty(t, got)
	_, err = s.Infer(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = s.Infer(nil) //nolint:staticcheck
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestInferFacetedAndCheckInferred(t *testing.T) {
	// A bore that reads Faceted never matches.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: facetedCopy(t, doc, boredPlate(t, doc))},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, -2, 17)})
	got, err := s.Infer(t.Context())
	require.NoError(t, err)
	require.Empty(t, got)

	// Check lists the matches no declared joint covers: the lap's declared pin1~base is covered, so
	// the other three are listed.
	lap := lapSet(t)
	rep, err := lap.Check(t.Context())
	require.NoError(t, err)
	require.Empty(t, rep.Inferred, "every inferred pair is declared")

	doc2 := decad.New()
	partial := parts.NewSet(doc2)
	bore := func(z0 float64) *decad.Body {
		b := block(t, doc2, 0, 0, 20, 20, z0, 5)
		b, err := decad.Cut(t.Context(), b, cylinder(t, doc2, 5, 3.2, z0-1, 7))
		require.NoError(t, err)
		return b
	}
	mustAdd(t, partial,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: bore(0)},
		parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: bore(5)},
		parts.Part{Name: namePin1, Kind: parts.Manufactured, Body: cylinder(t, doc2, 5, 3, -2, 14)})
	mustJoin(t, partial, parts.Joint{Name: "lap", Kind: parts.FaceContact, A: nameTop, B: nameBase},
		parts.Joint{Name: jointPin1Base, Kind: parts.PinInBore, A: namePin1, B: nameBase})
	rep, err = partial.Check(t.Context())
	require.NoError(t, err)
	require.Equal(t, []string{"pin1~top"}, inferredNames(rep.Inferred))
}

func TestInferRotatedCopy(t *testing.T) {
	// The lap, turned 30 degrees about Y by Transform, keeps Cylinder tags on its pins and bores
	// (displaced retained tags): the same four matches are inferred, on the turned axis.
	doc := decad.New()
	bore := func(z0 float64) *decad.Body {
		b := block(t, doc, 0, 0, 20, 20, z0, 5)
		b, err := decad.Cut(t.Context(), b, cylinder(t, doc, 5, 3.2, z0-1, 7))
		require.NoError(t, err)
		return b
	}
	turn, err := r3.RotationAround(r3.Vec{X: 10, Y: 10, Z: 5}, r3.Vec{Y: 1}, units.Degrees(30))
	require.NoError(t, err)
	turned := func(b *decad.Body) *decad.Body {
		out, err := decad.Transform(t.Context(), b, turn)
		require.NoError(t, err)
		return out
	}
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: turned(bore(0))},
		parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: turned(bore(5))},
		parts.Part{Name: namePin1, Kind: parts.Manufactured, Body: turned(cylinder(t, doc, 5, 3, -2, 14))})
	got, err := s.Infer(t.Context())
	require.NoError(t, err)
	require.ElementsMatch(t, []string{jointPin1Base, "pin1~top"}, inferredNames(got))
	want := turn.ApplyDir(r3.Vec{Z: 1})
	for _, ij := range got {
		require.InDelta(t, 0.2, ij.Measured.Value.Base(), 1e-6)
		d := ij.Joint.Axis.Dir
		require.InDelta(t, 1.0, d.Dot(want)*d.Dot(want), 1e-9, "the axis follows the turn")
	}
}
