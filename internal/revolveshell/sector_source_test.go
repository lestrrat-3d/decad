package revolveshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// taperedHub is the hub of the public sector Union tests: one axis span, a
// split cap, and the root cone from (5, 2.5) to (8, 4), recorded as segment
// 4, with its apex at the origin.
var taperedHub = []sectionrecord.Point2{{U: 6}, {U: 9}, {U: 9, V: 1.5},
	{U: 9, V: 3}, {U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}

const (
	taperedHubRootSpan = 3
	taperedHubRootRole = "side(0,4)"
)

// faceWithRole returns the one face of body that carries role.
func faceWithRole(t *testing.T, body *topo.Body, role string) *topo.Face {
	t.Helper()
	var found *topo.Face
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Role == role {
				require.Nil(t, found, "role %s names two faces", role)
				found = face
			}
		}
	}
	require.NotNil(t, found, "no face carries role %s", role)
	return found
}

// lineSegments copies body's profile record, which the fixtures build from
// whole lines only.
func lineSegments(t *testing.T, body *topo.Body) []sectionrecord.LineSeg {
	t.Helper()
	rp := body.Payload.(revolveplan.Payload)
	out := make([]sectionrecord.LineSeg, len(rp.Profile.Outer.Segments))
	for i, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		require.True(t, ok)
		out[i] = line
	}
	return out
}

// withLines replaces body's Revolve profile record with lines.
func withLines(body *topo.Body, lines []sectionrecord.LineSeg) {
	rp := body.Payload.(revolveplan.Payload)
	segments := make([]sectionrecord.CurveSegment, len(lines))
	for i, line := range lines {
		segments[i] = line
	}
	rp.Profile.Outer.Segments = segments
	body.Payload = rp
}

// wallFor is the wall the Revolve record sweeps from a to b, asked of the
// record with the walk and kind SpanFace uses.
func wallFor(t *testing.T, body *topo.Body, a, b sectionrecord.Point2) (surfacegeom.Surface, bool) {
	t.Helper()
	rp := body.Payload.(revolveplan.Payload)
	dz, dr := b.U-a.U, b.V-a.V
	kind := revolveaxis.WallCone
	if dz == 0 {
		kind = revolveaxis.WallPlane
	} else if dr == 0 {
		kind = revolveaxis.WallCylinder
	}
	surface, reversed, err := rp.WallSurface(rp.Basis(), survey2d.SegmentWalk{StartU: a.U, StartV: a.V,
		EndU: b.U, EndV: b.V, TanInU: dz, TanInV: dr}, kind)
	require.NoError(t, err)
	return surface, reversed
}

func TestRecordedSection(t *testing.T) {
	t.Run("accepts the tapered hub", func(t *testing.T) {
		spans, axis, _, ok := revolveshell.RecordedSection(revolveFixture(t, taperedHub))
		require.True(t, ok)
		require.Equal(t, 0, axis)
		require.Len(t, spans, 6)
	})
	t.Run("refuses three corners", func(t *testing.T) {
		// Four recorded lines, but (4, 0) is collinear: three corners.
		hub := revolveFixture(t, []sectionrecord.Point2{{U: 2}, {U: 4}, {U: 6}, {U: 2, V: 2}})
		require.Len(t, lineSegments(t, hub), 4)
		_, _, _, ok := revolveshell.RecordedSection(hub)
		require.False(t, ok)
	})
	t.Run("refuses a second axis span", func(t *testing.T) {
		hub := revolveFixture(t, []sectionrecord.Point2{{U: 2}, {U: 4}, {U: 5, V: 1}, {U: 6}, {U: 8},
			{U: 8, V: 2}, {U: 2, V: 2}})
		_, _, _, ok := revolveshell.RecordedSection(hub)
		require.False(t, ok)
	})
	t.Run("refuses an open loop record", func(t *testing.T) {
		// Only one segment's end moves, so every start and every face still
		// matches; the loop no longer closes.
		hub := revolveFixture(t, taperedHub)
		lines := lineSegments(t, hub)
		require.Equal(t, sectionrecord.Point2{U: 8, V: 4}, lines[3].End)
		lines[3].End.V += 0.25
		withLines(hub, lines)
		_, _, _, ok := revolveshell.RecordedSection(hub)
		require.False(t, ok)
	})
	t.Run("refuses a corner below the axis", func(t *testing.T) {
		// Move the corner (6, 1.5) to (6, -1.5) in the record and give its two
		// sides the walls the record now sweeps, so only the radius sign can
		// refuse.
		hub := revolveFixture(t, taperedHub)
		lines := lineSegments(t, hub)
		require.Equal(t, sectionrecord.Point2{U: 6, V: 1.5}, lines[5].End)
		moved := sectionrecord.Point2{U: 6, V: -1.5}
		lines[5].End, lines[6].Start = moved, moved
		withLines(hub, lines)
		for _, side := range []struct {
			role string
			a, b sectionrecord.Point2
		}{{"side(0,5)", lines[5].Start, moved}, {"side(0,6)", moved, lines[6].End}} {
			face := faceWithRole(t, hub, side.role)
			face.Surface, face.Reversed = wallFor(t, hub, side.a, side.b)
		}
		_, _, _, ok := revolveshell.RecordedSection(hub)
		require.False(t, ok)
	})
}

func TestSpanFace(t *testing.T) {
	rootSpan := func(t *testing.T, hub *topo.Body) facetedrecord.RevolveSectionSpan {
		t.Helper()
		spans, _, _, ok := revolveshell.RecordedSection(hub)
		require.True(t, ok)
		return spans[taperedHubRootSpan]
	}
	t.Run("binds the root span to its face", func(t *testing.T) {
		hub := revolveFixture(t, taperedHub)
		face, ok := revolveshell.SpanFace(hub, rootSpan(t, hub))
		require.True(t, ok)
		require.Same(t, faceWithRole(t, hub, taperedHubRootRole), face)
	})
	for _, tc := range []struct {
		name   string
		mutate func(hub *topo.Body, face *topo.Face)
	}{
		{"refuses the opposite orientation", func(_ *topo.Body, face *topo.Face) {
			face.Reversed = !face.Reversed
		}},
		{"refuses an origin from another producer", func(_ *topo.Body, face *topo.Face) {
			origins := append([]provenance.Ref(nil), face.Origins...)
			origins[0].Producer++
			face.Origins = origins
		}},
		{"refuses two matching faces", func(hub *topo.Body, face *topo.Face) {
			shell := hub.Lumps[0].Shells[0]
			shell.Faces = append(shell.Faces, &topo.Face{Surface: face.Surface,
				Reversed: face.Reversed, Origins: face.Origins, Body: hub})
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			hub := revolveFixture(t, taperedHub)
			span := rootSpan(t, hub)
			tc.mutate(hub, faceWithRole(t, hub, taperedHubRootRole))
			_, ok := revolveshell.SpanFace(hub, span)
			require.False(t, ok)
			_, _, _, ok = revolveshell.RecordedSection(hub)
			require.False(t, ok)
		})
	}
}

func TestRootCandidates(t *testing.T) {
	t.Run("certifies the tapered hub root", func(t *testing.T) {
		candidates := revolveshell.RootCandidates(revolveFixture(t, taperedHub))
		require.Len(t, candidates, 1)
		require.Equal(t, taperedHubRootSpan, candidates[0].RootSpan)
	})
	t.Run("skips a cone that ends on the axis", func(t *testing.T) {
		// The record's float apex of the side from (3, 0.6) to (1.2, 0) lands
		// below 1.2, so only the axis-endpoint gate keeps it out.
		hub := revolveFixture(t, []sectionrecord.Point2{{U: 1.2}, {U: 5}, {U: 5, V: 0.6}, {U: 3, V: 0.6}})
		spans, _, _, ok := revolveshell.RecordedSection(hub)
		require.True(t, ok)
		cone, isCone := spans[3].Face.Surface.(surfacegeom.Cone)
		require.True(t, isCone)
		require.Equal(t, sectionrecord.Point2{U: 1.2}, spans[3].End)
		require.Less(t, cone.Origin.X, spans[3].End.U)
		require.Empty(t, revolveshell.RootCandidates(hub))
	})
	t.Run("charges the meridian gap at both root ends", func(t *testing.T) {
		hub := revolveFixture(t, []sectionrecord.Point2{{U: 2}, {U: 8}, {U: 8, V: 2}, {U: 5.1, V: 1.3},
			{U: 2, V: 1.3}})
		candidates := revolveshell.RootCandidates(hub)
		require.Len(t, candidates, 1)
		c := candidates[0]
		angle, err := c.RootCone.HalfAngle.In(units.Radian)
		require.NoError(t, err)
		slope := math.Tan(angle)
		residual := max(revolveproof.RevolveMeridianResidual(c.RootStart, slope, c.RootCone.Origin.X),
			revolveproof.RevolveMeridianResidual(c.RootEnd, slope, c.RootCone.Origin.X))
		require.Positive(t, residual)
		require.GreaterOrEqual(t, c.ConeGap, residual)
	})
}

func TestCertifySector(t *testing.T) {
	translated, err := r3.Translation(r3.Vec{Z: 0.5})
	require.NoError(t, err)
	t.Run("certifies the radial lug on the tapered hub", func(t *testing.T) {
		cert, ok := revolveshell.CertifySector(revolveFixture(t, taperedHub), sectorFixture(t, 4.3))
		require.True(t, ok)
		require.Equal(t, taperedHubRootSpan, cert.Section.RootSpan)
	})
	t.Run("refuses two certified roots", func(t *testing.T) {
		// The spans (12, 6)-(11, 5.5) and (8, 4)-(4, 2) lie on one cone, so the
		// lug certifies against both roots.
		hub := revolveFixture(t, []sectionrecord.Point2{{U: 4}, {U: 12}, {U: 12, V: 6}, {U: 11, V: 5.5},
			{U: 10, V: 6}, {U: 8, V: 4}, {U: 4, V: 2}})
		sector := sectorFixture(t, 4.3)
		base := sectorPayload(sector).PointCone.Base
		certified := 0
		for _, section := range revolveshell.RootCandidates(hub) {
			if _, ok := revolveproof.CertifyRevolveConnectorBoundary(base.PointSection, section, base); ok {
				certified++
			}
		}
		require.Equal(t, 2, certified)
		_, ok := revolveshell.CertifySector(hub, sector)
		require.False(t, ok)
	})
	for _, tc := range []struct {
		name   string
		mutate func(p *facetedrecord.Payload)
	}{
		{"refuses a placed sector", func(p *facetedrecord.Payload) { p.Xform = translated }},
		{"refuses a placed base", func(p *facetedrecord.Payload) {
			trim := *p.PointCone
			trim.Base.Xform = translated
			p.PointCone = &trim
		}},
		{"refuses an apex off the origin", func(p *facetedrecord.Payload) {
			trim := *p.PointCone
			section := *trim.Base.PointSection
			section.Apex = r3.Vec{Z: 0.5}
			trim.Base.PointSection = &section
			p.PointCone = &trim
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			hub := revolveFixture(t, taperedHub)
			sector := sectorFixture(t, 4.3)
			p := sectorPayload(sector)
			tc.mutate(&p)
			sector.Payload = p
			_, ok := revolveshell.CertifySector(hub, sector)
			require.False(t, ok)
		})
	}
}
