package coilshell_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// squareCoil builds a coil of the square u ∈ [5, 7], v ∈ [-1, 1] about the
// plane's V axis: pitch 4 mm, 1.5 turns, right-handed.
func squareCoil(t *testing.T) (*topo.Body, coilshell.Payload) {
	t.Helper()
	pts := []sectionrecord.Point2{{U: 5, V: -1}, {U: 7, V: -1}, {U: 7, V: 1}, {U: 5, V: 1}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	line := revolveaxis.Line2{DV: 1}
	const pitch, turns = 4.0, 1.5
	side, err := coil.ProfileSide(t.Context(), profile, line, pitch, turns, freeform.NewFreeformWork())
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := coilshell.BuildBody(t.Context(), env, coilshell.Payload{
		Profile: profile, Frame: frame, Line: line, Side: side,
		Pitch: pitch, Turns: turns, Xform: r3.Identity(),
	})
	require.NoError(t, err)
	cp, ok := body.Payload.(coilshell.Payload)
	require.True(t, ok)
	return body, cp
}

// pi is π to 76 digits.
var pi, _ = new(big.Float).SetPrec(256).SetString("3.141592653589793238462643383279502884197169399375105820974944592307816406286")

// volumeGap is how far the held mesh's signed volume, over the exact
// rationals of its float coordinates, sits from want.
func volumeGap(m *topo.Mesh, want *big.Float) float64 {
	total := new(big.Rat)
	minor := func(a, b, c, d *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a, d), new(big.Rat).Mul(b, c))
	}
	for _, tri := range m.Triangles {
		var p [3][3]*big.Rat
		for k, vi := range tri {
			v := m.Vertices[vi]
			p[k] = [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
		}
		det := new(big.Rat).Mul(p[0][0], minor(p[1][1], p[1][2], p[2][1], p[2][2]))
		det.Sub(det, new(big.Rat).Mul(p[0][1], minor(p[1][0], p[1][2], p[2][0], p[2][2])))
		det.Add(det, new(big.Rat).Mul(p[0][2], minor(p[1][0], p[1][1], p[2][0], p[2][1])))
		total.Add(total, det)
	}
	got := new(big.Float).SetPrec(256).SetRat(total.Quo(total, big.NewRat(6, 1)))
	gap, _ := got.Sub(got, want).Abs(got).Float64()
	return gap
}

func TestTessellateRestatesTheHeldShell(t *testing.T) {
	t.Parallel()

	t.Run("VerifyAll", func(t *testing.T) {
		t.Parallel()
		body, cp := squareCoil(t)
		m, err := coilshell.Tessellate(t.Context(), body, cp, cp.Delta, true)
		require.NoError(t, err)
		require.Equal(t, cp.Verts, m.Vertices)
		require.Equal(t, cp.Tris, m.Triangles)
		require.Equal(t, cp.VertexBound, m.VertexBound)
		require.Equal(t, cp.Delta, m.Bound)
		require.NoError(t, tessellation.RequireClosedMesh(m.Triangles))

		// Each wall facet pair sweeps one chord cell of the station chain;
		// the facet names the side(i, k) face of the recorded segment that
		// chord lies on. The caps' facets follow the walls in held order.
		rec, err := cp.Record()
		require.NoError(t, err)
		stride := len(rec.Pts)
		roles := map[string]int{}
		for _, f := range m.Source {
			require.Len(t, f.Origins, 1)
			roles[f.Origins[0].Role]++
		}
		capCount := roles[provenance.RoleCapStart]
		require.Positive(t, capCount)
		require.Equal(t, capCount, roles[provenance.RoleCapEnd])
		for k := range 4 {
			require.Positive(t, roles[fmt.Sprintf("side(0,%d)", k)], "segment %d", k)
		}
		walls := len(m.Triangles) - 2*capCount
		for i, f := range m.Source {
			role := f.Origins[0].Role
			switch {
			case i < walls:
				seg := rec.Profile.Segments[rec.Profile.Chord[(i/2)%stride].Segment]
				require.Equal(t, fmt.Sprintf("side(%d,%d)", seg.Loop, seg.Index), role, "wall facet %d", i)
			case i < walls+capCount:
				require.Equal(t, provenance.RoleCapStart, role)
			default:
				require.Equal(t, provenance.RoleCapEnd, role)
			}
		}

		// Pappus over the screw motion: 1.5 turns of a 4 mm² square whose
		// centroid sits 6 mm from the axis sweeps 1.5·2π·6·4 mm³.
		want := new(big.Float).SetPrec(256).Mul(pi, big.NewFloat(1.5*2*6*4))
		require.True(t, m.SymDiffOK)
		require.LessOrEqual(t, volumeGap(m, want), m.VolSymDiff)
		require.Less(t, m.VolSymDiff, 0.01*1.5*2*math.Pi*6*4, "the volume proof is not vacuous")
		slack, volSymDiff, err := coilshell.MeshProofs(t.Context(), rec, coilshell.MeshInput{
			Verts: cp.Verts, Tris: cp.Tris, Delta: cp.Delta, MaxRound: cp.MaxRound,
		}, walls)
		require.NoError(t, err)
		require.Positive(t, slack)
		require.Equal(t, slack, m.AreaSlack)
		require.Equal(t, volSymDiff, m.VolSymDiff)
		beta, err := topobuild.VertexBounds(m)
		require.NoError(t, err)
		require.Equal(t, cp.VertexBound, beta)
	})

	t.Run("BelowVerifyAllSkipsTheProofs", func(t *testing.T) {
		t.Parallel()
		body, cp := squareCoil(t)
		m, err := coilshell.Tessellate(t.Context(), body, cp, cp.Delta, false)
		require.NoError(t, err)
		require.False(t, m.SymDiffOK)
		require.Zero(t, m.VolSymDiff)
		require.Zero(t, m.AreaSlack)
		require.Equal(t, cp.Delta, m.Bound)
	})

	t.Run("FinerThanDeltaRefuses", func(t *testing.T) {
		t.Parallel()
		body, cp := squareCoil(t)
		require.Positive(t, cp.Delta)
		_, err := coilshell.Tessellate(t.Context(), body, cp, cp.Delta/2, true)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a coil restates its held vertices")
	})

	t.Run("MissingFaceRefuses", func(t *testing.T) {
		t.Parallel()
		body, cp := squareCoil(t)
		for _, f := range topobuild.Faces(body) {
			if len(f.Origins) == 1 && f.Origins[0].Role == provenance.RoleCapEnd {
				f.Origins = nil
			}
		}
		_, err := coilshell.Tessellate(t.Context(), body, cp, cp.Delta, true)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "no face for role")
	})
}
