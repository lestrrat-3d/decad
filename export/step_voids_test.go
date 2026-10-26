package export_test

import (
	"bytes"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/step-export-design.md §9's tests for a solid with a
// cavity: a 20 mm cube with a 10 mm cubic cavity at its middle, written as a
// BREP_WITH_VOIDS.

// cavityCube is the 20 mm cube at the origin with the cube from (5, 5, 5) to (15, 15, 15)
// cut out of its middle, so it has an outer shell and one void shell.
func cavityCube(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	outer := decadtest.NewBlock(t, doc, 0, 0, 20, 20, units.Millimeters(20))
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(5, 5, 15, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	inner, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	part, err := decad.Cut(t.Context(), outer, inner)
	require.NoError(t, err)
	require.Len(t, part.Shells(), 2)
	return part
}

func voidsHeader() step.Header {
	return step.Header{
		Name: "cavity", Authors: []string{"a"}, Organizations: []string{"o"},
		Description: []string{"d"}, PreprocessorVersion: "p", OriginatingSystem: "s",
		Timestamp: time.Date(2026, 1, 2, 3, 4, 5, 0, time.UTC),
	}
}

// stepIndex maps the IDs of a file's entities to them.
func stepIndex(f step.File) map[step.Reference]step.Entity {
	byID := map[step.Reference]step.Entity{}
	for _, e := range f.Entities {
		byID[step.Reference(e.ID)] = e
	}
	return byID
}

// faceNormalAndPoint returns the plane normal and a point of the ADVANCED_FACE e.
func faceNormalAndPoint(t *testing.T, byID map[step.Reference]step.Entity, e step.Entity) (r3.Vec, r3.Vec) {
	t.Helper()
	plane := byID[e.Parameters[2].(step.Reference)]
	placement := byID[plane.Parameters[1].(step.Reference)]
	vec := func(ref step.Value) r3.Vec {
		list := byID[ref.(step.Reference)].Parameters[1].(step.List)
		return r3.NewVec(float64(list[0].(step.Real)), float64(list[1].(step.Real)), float64(list[2].(step.Real)))
	}
	return vec(placement.Parameters[2]), vec(placement.Parameters[1])
}

func TestNewSTEPFileWritesACavityAsBrepWithVoids(t *testing.T) {
	t.Parallel()
	f, err := export.NewSTEPFile(t.Context(), cavityCube(t), units.Millimeters(0.1), voidsHeader())
	require.NoError(t, err)
	counts := map[string]int{}
	for _, e := range f.Entities {
		counts[e.Name]++
	}
	require.Equal(t, 1, counts["BREP_WITH_VOIDS"])
	require.Equal(t, 1, counts["ORIENTED_CLOSED_SHELL"])
	require.Equal(t, 2, counts["CLOSED_SHELL"])
	require.Zero(t, counts["MANIFOLD_SOLID_BREP"])
	require.Equal(t, 24, counts["ADVANCED_FACE"], "twelve triangles for the outer cube and twelve for the cavity")
	data, err := f.Marshal()
	require.NoError(t, err)
	require.Contains(t, string(data), "BREP_WITH_VOIDS")
	require.Contains(t, string(data), "ORIENTED_CLOSED_SHELL")
}

func TestNewSTEPFileCavityShellsFaceAwayFromTheMaterial(t *testing.T) {
	t.Parallel()
	f, err := export.NewSTEPFile(t.Context(), cavityCube(t), units.Millimeters(0.1), voidsHeader())
	require.NoError(t, err)
	byID := stepIndex(f)
	centre := r3.NewVec(10, 10, 10)
	var brep step.Entity
	for _, e := range f.Entities {
		if e.Name == "BREP_WITH_VOIDS" {
			brep = e
		}
	}
	require.NotZero(t, brep.ID)
	// Every face's normal points away from the material: out of the cube on the outer
	// shell, and toward the cavity's middle on the void shell.
	faceCheck := func(shell step.Reference, outward bool) {
		for _, ref := range byID[shell].Parameters[1].(step.List) {
			normal, point := faceNormalAndPoint(t, byID, byID[ref.(step.Reference)])
			side := normal.Dot(point.Sub(centre))
			if outward {
				require.Positive(t, side, "an outer face points away from the cube's middle")
			} else {
				require.Negative(t, side, "a cavity face points toward the cavity's middle")
			}
		}
	}
	faceCheck(brep.Parameters[1].(step.Reference), true)
	voids := brep.Parameters[2].(step.List)
	require.Len(t, voids, 1)
	oriented := byID[voids[0].(step.Reference)]
	require.Equal(t, "ORIENTED_CLOSED_SHELL", oriented.Name)
	require.Equal(t, step.Enumeration("T"), oriented.Parameters[3])
	faceCheck(oriented.Parameters[2].(step.Reference), false)
}

func TestSTEPCavityIsDeterministic(t *testing.T) {
	t.Parallel()
	part := cavityCube(t)
	var a, b bytes.Buffer
	opts := []export.STEPOption{export.WithSTEPName("cavity"), export.WithSTEPAuthor("a"), export.WithSTEPOrganization("o"),
		export.WithSTEPTimestamp(time.Date(2026, 1, 2, 3, 4, 5, 0, time.UTC))}
	require.NoError(t, export.STEP(t.Context(), &a, part, units.Millimeters(0.1), opts...))
	require.NoError(t, export.STEP(t.Context(), &b, part, units.Millimeters(0.1), opts...))
	require.Equal(t, a.Bytes(), b.Bytes())
	require.Contains(t, a.String(), "BREP_WITH_VOIDS")
}

func TestNewSTEPFileWritesAClosedShellAsBrepWithVoids(t *testing.T) {
	t.Parallel()
	cube := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20))
	closed, err := decad.Shell(t.Context(), cube, nil, units.Millimeters(3), decad.WithNoOpenings())
	require.NoError(t, err)
	f, err := export.NewSTEPFile(t.Context(), closed, units.Millimeters(0.1), voidsHeader())
	require.NoError(t, err)
	counts := map[string]int{}
	for _, e := range f.Entities {
		counts[e.Name]++
	}
	require.Equal(t, 1, counts["BREP_WITH_VOIDS"])
	require.Equal(t, 1, counts["ORIENTED_CLOSED_SHELL"])
	require.Equal(t, 2, counts["CLOSED_SHELL"])
	byID := stepIndex(f)
	for _, e := range f.Entities {
		if e.Name != "BREP_WITH_VOIDS" {
			continue
		}
		// The void is the 14 mm cube at the middle: its faces point toward (10, 10, 10).
		oriented := byID[e.Parameters[2].(step.List)[0].(step.Reference)]
		shell := byID[oriented.Parameters[2].(step.Reference)]
		for _, ref := range shell.Parameters[1].(step.List) {
			normal, point := faceNormalAndPoint(t, byID, byID[ref.(step.Reference)])
			require.Negative(t, normal.Dot(point.Sub(r3.NewVec(10, 10, 10))))
		}
	}
}

// shellPoints returns the placement point of every face of the CLOSED_SHELL ref.
func shellPoints(t *testing.T, byID map[step.Reference]step.Entity, shell step.Reference) []r3.Vec {
	t.Helper()
	var out []r3.Vec
	for _, ref := range byID[shell].Parameters[1].(step.List) {
		_, point := faceNormalAndPoint(t, byID, byID[ref.(step.Reference)])
		out = append(out, point)
	}
	require.NotEmpty(t, out)
	return out
}

func requirePointsInBox(t *testing.T, points []r3.Vec, lo, hi r3.Vec) {
	t.Helper()
	for _, p := range points {
		require.True(t, p.X >= lo.X && p.X <= hi.X && p.Y >= lo.Y && p.Y <= hi.Y && p.Z >= lo.Z && p.Z <= hi.Z,
			"a shell point lies outside its lump's box")
	}
}

// A lump with a cavity beside a lump without one: each cavity pairs with the
// outer shell of its own lump.
func TestNewSTEPFilePairsACavityWithItsOwnLump(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	closed, err := decad.Shell(t.Context(), decadtest.NewBlock(t, doc, 0, 0, 20, 20, units.Millimeters(20)), nil,
		units.Millimeters(3), decad.WithNoOpenings())
	require.NoError(t, err)
	beside := decadtest.NewBlock(t, doc, 40, 0, 50, 10, units.Millimeters(10))
	both, err := decad.Union(t.Context(), closed, beside)
	require.NoError(t, err)
	require.Len(t, both.Lumps(), 2)

	f, err := export.NewSTEPFile(t.Context(), both, units.Millimeters(0.1), voidsHeader())
	require.NoError(t, err)
	byID := stepIndex(f)
	var brep, solid step.Entity
	for _, e := range f.Entities {
		switch e.Name {
		case "BREP_WITH_VOIDS":
			brep = e
		case "MANIFOLD_SOLID_BREP":
			solid = e
		}
	}
	require.NotZero(t, brep.ID)
	require.NotZero(t, solid.ID)
	requirePointsInBox(t, shellPoints(t, byID, brep.Parameters[1].(step.Reference)),
		r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 20))
	voids := brep.Parameters[2].(step.List)
	require.Len(t, voids, 1)
	oriented := byID[voids[0].(step.Reference)]
	requirePointsInBox(t, shellPoints(t, byID, oriented.Parameters[2].(step.Reference)),
		r3.NewVec(3, 3, 3), r3.NewVec(17, 17, 17))
	requirePointsInBox(t, shellPoints(t, byID, solid.Parameters[1].(step.Reference)),
		r3.NewVec(40, 0, 0), r3.NewVec(50, 10, 10))
}
