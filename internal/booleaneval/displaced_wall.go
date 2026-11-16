package booleaneval

import (
	"context"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// displacedWallPlane is rule DW of agent-docs/carrier-record-design.md §8.7 and
// agent-docs/faceted-source-retention-design.md §11: it states, on the retained
// group of a side wall of a displaced loop, how far the face's true piece lies
// from the tag.
//
// A mesh Boolean copies each operand face's surface onto its facets as a
// retained tag (facetedrecord.SourceTag reads the face alone). The recorded
// Plane or Cylinder of a side wall of a displaced loop lies only within the
// loop's bound b_l of the wall its section denotes, so a tag that states
// NormalBound 0 and no AxialDelta would claim the recorded surface IS the
// denoted one. Where b_l > 0 a Plane tag takes AxialDelta = max(AxialDelta,
// b_l), which the retained-plane slab charges, and a Cylinder tag is declined:
// it states no surface and is not retained. Every other face keeps its tag.
//
// The loop a face names is read from its roles (side(l,j), fillet(l,j),
// chamfer(l,j)). A cap face and a chamfer band patch take no bound; any other
// face whose roles name no single loop takes the largest loop bound. The loop
// bound is the DisplacedPayload's LoopDelta[l], or, for a prism with a section
// displacement, the h_loop of its valid owned carrier record on a loop its
// ExactCarriers listing does not cover, else its SectionDelta.
func displacedWallPlane(group facetedrecord.Group, owner *topo.Body, f *topo.Face) facetedrecord.Group {
	if !group.Retained || group.Surface == nil {
		return group
	}
	kind := group.Surface.Kind()
	if kind != surfacegeom.KindPlane && kind != surfacegeom.KindCylinder {
		return group
	}
	bounds := loopBounds(owner)
	if bounds == nil {
		return group
	}
	b, named := faceLoopBound(f, bounds)
	if !named || !(b > 0) {
		return group
	}
	if kind == surfacegeom.KindCylinder {
		group.Surface, group.Retained, group.Reversed, group.NormalBound = nil, false, false, 0
		group.CapRecord, group.AxialDelta, group.HasAxialDelta = nil, 0, false
		return group
	}
	group.AxialDelta, group.HasAxialDelta = max(group.AxialDelta, b), true
	return group
}

// loopBounds is the per-loop bound b_l of a displaced owner, or nil when the
// owner is not displaced.
func loopBounds(owner *topo.Body) []float64 {
	switch pl := owner.Payload.(type) {
	case capblend.DisplacedPayload:
		return pl.LoopDelta
	case prismrecord.Payload:
		if !(pl.SectionDelta > 0) {
			return nil
		}
		loops := 1 + len(pl.Profile.Holes)
		bounds := make([]float64, loops)
		budget := proofbound.NewWorkBudget(context.Background())
		cr, valid, err := prismrecord.CarrierRecordOf(budget, owner)
		if err != nil || !valid {
			valid = false
		}
		for l := range bounds {
			bounds[l] = pl.SectionDelta
			exact, err := pl.CarrierExact(budget, l)
			if err == nil && exact {
				bounds[l] = 0
				continue
			}
			if valid {
				bounds[l] = min(pl.SectionDelta, cr.LoopH(l))
			}
		}
		return bounds
	default:
		return nil
	}
}

// faceLoopBound is the bound a face's roles name: that of the one loop they
// name, the largest when they name several or none, and named=false for a cap
// face or a chamfer band patch, which take none.
func faceLoopBound(f *topo.Face, bounds []float64) (float64, bool) {
	loop, many := -1, false
	for _, o := range f.Origins {
		switch {
		case o.Role == provenance.RoleCapStart || o.Role == provenance.RoleCapEnd,
			strings.HasPrefix(o.Role, "chamferCap("):
			return 0, false
		}
		l, ok := roleLoop(o.Role)
		if !ok {
			continue
		}
		if loop >= 0 && l != loop {
			many = true
		}
		loop = l
	}
	if loop < 0 || many || loop >= len(bounds) {
		return maxFloat(bounds), true
	}
	return bounds[loop], true
}

// roleLoop is the loop index of a side(l,j), fillet(l,j) or chamfer(l,j) role.
func roleLoop(role string) (int, bool) {
	for _, prefix := range [...]string{"side(", "fillet(", "chamfer("} {
		rest, ok := strings.CutPrefix(role, prefix)
		if !ok {
			continue
		}
		digits, _, _ := strings.Cut(rest, ",")
		l, err := strconv.Atoi(digits)
		return l, err == nil && l >= 0
	}
	return 0, false
}

func maxFloat(xs []float64) float64 {
	m := 0.0
	for _, x := range xs {
		m = max(m, x)
	}
	return m
}
