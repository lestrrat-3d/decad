package booleaneval

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// replayPlacedRectangleDepartures reconstructs the same six unplaced tags
// that the ordinary prism rebuild reads, without rebuilding its topology.
func replayPlacedRectangleDepartures(ctx context.Context, body *topo.Body, mesh *topo.Mesh, faces []*topo.Face, b *proofbound.ReplayBudget) (placedDepartures, error) {
	var out placedDepartures
	if body == nil {
		return out, errBrakeEvalBoundedOperandIsMissing
	}
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok || b == nil || mesh == nil || pp.SurfaceResult || !facetproof.TranslationOnly(pp.Xform) || len(faces) != 6 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 4 || pp.Walks == nil || !pp.Walks.Metered ||
		pp.Walks.Spent != 0 || pp.Walks.ReconstructionSpent != 0 || len(pp.Walks.Outer) != 4 || len(pp.Walks.Holes) != 0 ||
		len(pp.Walks.Profile.Holes) != 0 || len(pp.Walks.Profile.Outer.Segments) != 4 {
		return out, errBrakeEvalBoundedOperandTagPlacementRequiresARecordedRectangle
	}
	if err := b.SpendWork(ctx, 512); err != nil {
		return out, err
	}
	if err := b.ReserveRecords(ctx, 16); err != nil {
		return out, err
	}
	if err := b.ReserveBytes(ctx, 4096); err != nil {
		return out, err
	}
	for i, seg := range pp.Profile.Outer.Segments {
		if _, ok := seg.(sectionrecord.LineSeg); !ok {
			return out, errBrakeEvalBoundedOperandSectionIsNotLinear
		}
		if _, ok := pp.Walks.Profile.Outer.Segments[i].(sectionrecord.LineSeg); !ok {
			return out, errBrakeEvalBoundedOperandWalkIsNotLinear
		}
	}
	if !pp.Walks.Reusable(pp.Profile) {
		return out, errBrakeEvalBoundedOperandWalksChanged
	}
	// Adjacent axis-aligned line tangents cannot coalesce in BuildLoopSides.
	axis := func(u, v float64) int {
		if math.IsNaN(u) || math.IsNaN(v) || math.IsInf(u, 0) || math.IsInf(v, 0) {
			return -1
		}
		if u != 0 && v == 0 {
			return 0
		}
		if v != 0 && u == 0 {
			return 1
		}
		return -1
	}
	for i, w := range pp.Walks.Outer {
		a, c := axis(w.TanInU, w.TanInV), axis(w.TanOutU, w.TanOutV)
		next := pp.Walks.Outer[(i+1)%4]
		if w.Kind != survey2d.WalkLine || w.Closed || a < 0 || a != c || c == axis(next.TanInU, next.TanInV) {
			return out, errBrakeEvalBoundedOperandRectangleWalkIsUnavailable
		}
	}
	if _, err := topobuild.VertexBoundsReplay(ctx, mesh, b); err != nil {
		return out, err
	}
	if err := b.SpendWork(ctx, uint64(len(mesh.Vertices))); err != nil {
		return out, err
	}
	for _, a := range [][2]uint64{{uint64(len(faces)), uint64(unsafe.Sizeof(float64(0)))}, {uint64(len(faces)), uint64(unsafe.Sizeof(false))}} {
		if err := b.ReserveRecords(ctx, a[0]); err != nil {
			return out, err
		}
		if err := b.ReserveArray(ctx, a[0], a[1]); err != nil {
			return out, err
		}
	}
	out.departure = make([]float64, len(faces))
	out.stated = make([]bool, len(faces))
	source := pp
	source.Xform = r3.Identity()
	var tags [6]surfacegeom.Plane
	for i, w := range pp.Walks.Outer {
		midpoint := source.Point((w.StartU+w.EndU)/2, (w.StartV+w.EndV)/2, source.Z0)
		frame, err := r3.NewFrame(midpoint, source.Dir(w.TanInU, w.TanInV, 0), source.Dir(0, 0, 1))
		if err != nil {
			return placedDepartures{}, err
		}
		tags[i] = surfacegeom.Plane{Frame: frame}
	}
	for i, capItem := range []struct {
		z    float64
		flip bool
	}{{source.Z0, true}, {source.Z1, false}} {
		f, err := source.CapFrame(capItem.z, capItem.flip)
		if err != nil {
			return placedDepartures{}, err
		}
		tags[4+i] = surfacegeom.Plane{Frame: f}
	}
	for i, f := range faces {
		if f == nil {
			return placedDepartures{}, errBrakeEvalBoundedOperandFaceMissing
		}
		moved, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			continue
		}
		d, err := facetedrecord.ReplayPlacedBuiltPlaneDeparture(ctx, tags[i], moved, pp.Xform, b)
		if err != nil {
			return placedDepartures{}, err
		}
		out.departure[i], out.stated[i] = d, true
	}
	return out, nil
}
