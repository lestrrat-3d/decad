package cupwall

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the cup of docs/modify-design.md §9 (Table B, B5/B6). A cup
// is the outer region O on its interval less the cavity region C on its own,
// sharing a rim at the open end and a floor at the closed one. Its record is a
// two-slab stackedrecord.Payload (docs/modify-reach-design.md §9.1): a floor
// slab over O and a wall slab over the bands between O's loops and C's, with C
// exposed between them as the pocket floor. The stacked build makes its body,
// and every cup survey, mesh and mass reading reads the View the record
// re-derives. It re-evaluates under Transform (evaluator §8).
//
// A holed section (k ≥ 1 posts in the pocket) builds too: the outer region and
// the cavity region each carry k holes, so the wall slab holds 1 + k bands, one
// wrapping each post. The cavity is a VOID the solid encloses, so its loops
// invert — the void's outer boundary is a hole in the solid, each of the
// void's own holes a solid post — and every band still hangs off the one floor
// slab, so the result is one lump (Table B, B5/B6).

// Sense is the wall sense of a cup's shell: Inward grows the wall into the
// receiver, Outward off it. Its values match the root package's ShellSense.
type Sense int

const (
	// Inward grows the wall into the original solid.
	Inward Sense = iota
	// Outward grows the wall off the original solid.
	Outward
)

// View is a cup read as two prisms, re-derived from its stacked record
// (Payload.View) for every cup survey, mesh and mass reading, and the form
// the shell assembles before recording it (View.Payload): the outer region
// O and the cavity region C (each walked in its natural sense — the outer loop
// counter-clockwise), the plane frame, the three sweep planes, the private
// shell-morphology certificate and the accumulated rigid placement. The open
// end (the removed cap, where the rim is) is ZOpen; the outer prism's floor is
// at ZOuter, the cavity's floor (shellCap) at ZCav, which lies between the two
// so the floor slab is [ZOuter, ZCav] (docs/modify-design.md §9).
//
// ZOpenDelta, ZOuterDelta and ZCavDelta are the axial displacement of their
// matching levels. ZOpen and the unstepped floor level retain the displacement
// of the receiver end they came from. A derived floor level additionally carries
// the thickness conversion and float-sum rounding. Every level-derived cup
// reading uses its matching displacement — the two heights and the volume, area
// and centroid built on them, the box, and the wall vertices and vertical edges
// the shared prism build stamps.
//
// ThicknessDelta is the shell thickness's OWN conversion displacement — the
// bound extent.MagnitudeInBounded proved when it converted the caller's stated
// thickness into millimetres (shell.go). It is distinct from ZOuterDelta and
// ZCavDelta: those cover the derived FLOOR LEVEL's own float-sum rounding
// (which already folds ThicknessDelta in — ViewFor's step closure), and
// this one is the reading Evaluate publishes when the shell-wall theorem holds
// exactly on the payload's own morphology — the number the theorem's t is
// held in, not the theorem itself (docs/payload-verification-design.md §4.1).
//
// OffsetDelta is the OFFSET region's section displacement — the cavity
// inward, the outer region outward: the proven upper bound
// offset2d.OffsetSectionDelta states on how far any recorded boundary point of that
// region sits from the boundary of P ⊖ t* or P ⊕ t* the shell denotes, t* the
// caller's thickness in exact millimetres. It covers the thickness conversion
// and the offset solve's own rounding together. The other region is the
// receiver's own section and carries none. OuterPrism and CavityPrism hand it
// to the offset region's prism as its SectionDelta, so every view reading
// built on that region — Bounds, the mass path, the mesh — charges it there,
// and plan charges it to the offset region's columns of the stacked build
// (docs/modify-design.md §9, §10).
type View struct {
	Outer          momentinput.Profile
	Cavity         momentinput.Profile
	Frame          r3.Frame
	ZOpen          float64
	ZOuter         float64
	ZCav           float64
	ZOpenDelta     float64
	ZOuterDelta    float64
	ZCavDelta      float64
	Thickness      float64
	ThicknessDelta float64
	OffsetDelta    float64
	Sense          Sense
	Xform          r3.Transform
}

// AxialDelta reports the largest held-level displacement to a body-relative
// stop resolved against this cup's faces (stops.go).
func (cp View) AxialDelta() float64 {
	return math.Max(cp.ZOpenDelta, math.Max(cp.ZOuterDelta, cp.ZCavDelta))
}

// OpenScalar is the open end's sweep level as a bounded reading carrying its
// own axial displacement.
func (cp View) OpenScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.ZOpen, cp.ZOpenDelta)
}

// OuterScalar is the outer prism's floor level as a bounded reading.
func (cp View) OuterScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.ZOuter, cp.ZOuterDelta)
}

// CavityScalar is the pocket floor level as a bounded reading.
func (cp View) CavityScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.ZCav, cp.ZCavDelta)
}

// BasePrism shares the cup's frame and placement with the prism helpers that
// need no axial level.
func (cp View) BasePrism() prismrecord.Payload {
	return prismrecord.Payload{Frame: cp.Frame, Xform: cp.Xform}
}

// prismBetween builds an ordered prism from two cup levels, preserving each
// level's own axial displacement at the matching prism end.
func (cp View) prismBetween(a, b proofbound.BoundedScalar) prismrecord.Payload {
	if a.Value <= b.Value {
		return prismrecord.Payload{
			Frame: cp.Frame, Z0: a.Value, Z1: b.Value,
			Z0Delta: a.Bound, Z1Delta: b.Bound,
			Xform: cp.Xform,
		}
	}
	return prismrecord.Payload{
		Frame: cp.Frame, Z0: b.Value, Z1: a.Value,
		Z0Delta: b.Bound, Z1Delta: a.Bound,
		Xform: cp.Xform,
	}
}

// OuterPrism is the cup's outer prism, without its profile. The offset
// region's prism carries OffsetDelta as its section displacement: the outer
// one outward.
func (cp View) OuterPrism() prismrecord.Payload {
	pp := cp.prismBetween(cp.OuterScalar(), cp.OpenScalar())
	if cp.Sense == Outward {
		pp.SectionDelta = cp.OffsetDelta
	}
	return pp
}

// CavityPrism is the cup's cavity prism, without its profile. It carries
// OffsetDelta as its section displacement when the cavity is the offset
// region, inward.
func (cp View) CavityPrism() prismrecord.Payload {
	pp := cp.prismBetween(cp.CavityScalar(), cp.OpenScalar())
	if cp.Sense != Outward {
		pp.SectionDelta = cp.OffsetDelta
	}
	return pp
}

// ExtentAlong is the cup's extent interval along an arbitrary world direction
// g, beside the displacement its ends carry — the OUTER prism's reading
// (docs/modify-design.md §10, Table D, D5). The cavity is interior and reaches
// no farther than the outer region, so the outward extent is the solid outer
// prism's, read by the same prismrecord.Payload.ExtentAlong the outer prism's
// bounds already use. This is what a through-all stop consults when a cup is a
// live body in the sweep's path. An outward cup's outer region is the offset
// one: its recorded extent is read, and its OffsetDelta joins the displacement
// the ends carry — moving every boundary point by at most OffsetDelta moves an
// extreme along g by at most OffsetDelta·|g|, and |g| ≤ its L1 norm.
func (cp View) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	outer := cp.OuterPrism()
	outer.Profile = cp.Outer
	displaced := outer.SectionDelta
	outer.SectionDelta = 0
	lo, hi, delta, err := outer.ExtentAlong(g)
	if err != nil || displaced == 0 {
		return lo, hi, delta, err
	}
	return lo, hi, proofbound.AbsSumUpper(delta, proofbound.ProductUpper(displaced, stackedrecord.VecL1(g))), nil
}

// ViewFor assembles the cup record from the receiver prism, its offset
// section with that section's proven displacement, and the shell
// sense/opening (docs/modify-design.md §9). removedEnd
// opens the cup at the top (z1); a removed start opens it at the bottom (z0),
// its mirror. Inward, O is the original section P and C the erosion Q; outward,
// O is the dilation Q and C the original P.
func ViewFor(pp prismrecord.Payload, offset momentinput.Profile, s, t, tDelta, offsetDelta float64,
	removedEnd bool) View {
	z0, z1 := pp.Z0, pp.Z1
	o, c := pp.Profile, offset
	if s < 0 {
		o, c = offset, pp.Profile
	}
	sense := Inward
	if s < 0 {
		sense = Outward
	}
	cp := View{
		Outer:          o,
		Cavity:         c,
		Frame:          pp.Frame,
		Thickness:      t,
		ThicknessDelta: tDelta,
		OffsetDelta:    offsetDelta,
		Sense:          sense,
		Xform:          pp.Xform,
	}
	// step is the one derived floor level. It carries the source end's own
	// displacement, the thickness conversion, and this float sum's rounding.
	step := func(from, delta, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))
	}
	if removedEnd { // open at the top
		cp.ZOpen = z1
		cp.ZOpenDelta = pp.Z1Delta
		if s > 0 {
			cp.ZOuter, cp.ZOuterDelta = z0, pp.Z0Delta
			cp.ZCav, cp.ZCavDelta = step(z0, pp.Z0Delta, t)
		} else {
			cp.ZOuter, cp.ZOuterDelta = step(z0, pp.Z0Delta, -t)
			cp.ZCav, cp.ZCavDelta = z0, pp.Z0Delta
		}
	} else { // open at the bottom — the mirror
		cp.ZOpen = z0
		cp.ZOpenDelta = pp.Z0Delta
		if s > 0 {
			cp.ZOuter, cp.ZOuterDelta = z1, pp.Z1Delta
			cp.ZCav, cp.ZCavDelta = step(z1, pp.Z1Delta, -t)
		} else {
			cp.ZOuter, cp.ZOuterDelta = step(z1, pp.Z1Delta, t)
			cp.ZCav, cp.ZCavDelta = z1, pp.Z1Delta
		}
	}
	return cp
}

// Payload is the cup's stored record (docs/modify-reach-design.md §9.1): a
// two-slab stackedrecord.Payload plus the shell morphology certificate the
// cup's own surveys read (docs/payload-verification-design.md §4.1). The floor
// slab holds the outer region O over the floor interval; the wall slab holds
// the band between O's outer loop and the cavity's, then one band lining each
// hole, over the cavity interval; their interface exposes the cavity region C
// as the pocket floor. Every consumer that reads a cup reads its View, which
// re-derives the cup's regions and levels from the slabs. It implements
// topo.Payload.
//
// The morphology is the shell's own: Thickness and ThicknessDelta, the offset
// region's proven displacement OffsetDelta, and the Sense. OffsetDelta is
// charged loop by loop (plan), to the offset region's walls and to the
// planar faces those walls bound, never to the receiver's own loops.
type Payload struct {
	Stack          stackedrecord.Payload
	Thickness      float64
	ThicknessDelta float64
	OffsetDelta    float64
	Sense          Sense
}

// Transform is the accumulated rigid placement.
func (cp Payload) Transform() r3.Transform { return cp.Stack.Xform }

// Placed re-evaluates the same cup under the composed motion (evaluator §8).
func (cp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	cp.Stack.Xform = composed
	return Build(ctx, env, cp)
}

// AxialDelta reports the largest held-level displacement (stops.go).
func (cp Payload) AxialDelta() float64 { return cp.Stack.AxialDelta() }

// ExtentAlong is the view's outer-prism extent (View.ExtentAlong).
func (cp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	return cp.View().ExtentAlong(g)
}

// floorSlab is the index of the slab holding the outer region: the first
// when the cup opens at the top, the second when it opens at the bottom.
func (cp Payload) floorSlab() int {
	if len(cp.Stack.Slabs[0].Regions) == 1 && len(cp.Stack.Slabs[1].Regions) == 1 {
		// A hole-free cup: the floor is the slab whose region has no hole.
		if len(cp.Stack.Slabs[0].Regions[0].Holes) == 0 {
			return 0
		}
		return 1
	}
	if len(cp.Stack.Slabs[0].Regions) == 1 {
		return 0
	}
	return 1
}

// View re-derives the cup's regions and levels from its slabs: the outer
// region is the floor slab's one region, the cavity is the interface's one
// exposed record, the floor slab's far end is ZOuter, the interface is ZCav,
// and the wall slab's far end is ZOpen.
func (cp Payload) View() View {
	f := cp.floorSlab()
	floor, wall := cp.Stack.Slabs[f], cp.Stack.Slabs[1-f]
	boundary := cp.Stack.Interfaces[0]
	v := View{
		Outer:          floor.Regions[0],
		Frame:          cp.Stack.Frame,
		Thickness:      cp.Thickness,
		ThicknessDelta: cp.ThicknessDelta,
		OffsetDelta:    cp.OffsetDelta,
		Sense:          cp.Sense,
		Xform:          cp.Stack.Xform,
	}
	if f == 0 {
		v.Cavity = boundary.LowerExposed[0]
		v.ZOuter, v.ZOuterDelta = floor.Z0, floor.Z0Delta
		v.ZCav, v.ZCavDelta = floor.Z1, floor.Z1Delta
		v.ZOpen, v.ZOpenDelta = wall.Z1, wall.Z1Delta
	} else {
		v.Cavity = boundary.UpperExposed[0]
		v.ZOuter, v.ZOuterDelta = floor.Z1, floor.Z1Delta
		v.ZCav, v.ZCavDelta = floor.Z0, floor.Z0Delta
		v.ZOpen, v.ZOpenDelta = wall.Z0, wall.Z0Delta
	}
	return v
}

// Payload records the view as a stacked cup: the floor slab over O, the wall
// slab over the band between O's outer and C's (C's outer reversed as that
// band's one hole) and, per hole i of O, the band between C's hole i
// (reversed, an outer) and O's hole i, and the interface exposing C itself.
// The two regions must have one loop count; a dropped loop is S11a, caught
// before here.
func (cp View) Payload(ctx context.Context) (Payload, error) {
	bands, err := prismshell.ShellWallBands(ctx, cp.Outer, cp.Cavity)
	if err != nil {
		return Payload{}, err
	}
	floor := stackedrecord.Slab{Regions: []momentinput.Profile{cp.Outer}}
	wall := stackedrecord.Slab{Regions: bands}
	exposed := []momentinput.Profile{cp.Cavity}
	out := Payload{
		Stack:          stackedrecord.Payload{Frame: cp.Frame, Xform: cp.Xform},
		Thickness:      cp.Thickness,
		ThicknessDelta: cp.ThicknessDelta,
		OffsetDelta:    cp.OffsetDelta,
		Sense:          cp.Sense,
	}
	if cp.ZOpen > cp.ZOuter { // open at the top
		floor.Z0, floor.Z0Delta, floor.Z1, floor.Z1Delta = cp.ZOuter, cp.ZOuterDelta, cp.ZCav, cp.ZCavDelta
		wall.Z0, wall.Z0Delta, wall.Z1, wall.Z1Delta = cp.ZCav, cp.ZCavDelta, cp.ZOpen, cp.ZOpenDelta
		out.Stack.Slabs = []stackedrecord.Slab{floor, wall}
		out.Stack.Interfaces = []stackedrecord.Interface{{LowerExposed: exposed}}
	} else {
		wall.Z0, wall.Z0Delta, wall.Z1, wall.Z1Delta = cp.ZOpen, cp.ZOpenDelta, cp.ZCav, cp.ZCavDelta
		floor.Z0, floor.Z0Delta, floor.Z1, floor.Z1Delta = cp.ZCav, cp.ZCavDelta, cp.ZOuter, cp.ZOuterDelta
		out.Stack.Slabs = []stackedrecord.Slab{wall, floor}
		out.Stack.Interfaces = []stackedrecord.Interface{{UpperExposed: exposed}}
	}
	return out, nil
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "cupPayload" }

// MassRecords reads the two prism records whose difference forms the cup.
func (cp View) MassRecords() (massmoment.PrismRecord, massmoment.PrismRecord) {
	outer := cp.OuterPrism()
	outer.Profile = cp.Outer
	cavity := cp.CavityPrism()
	cavity.Profile = cp.Cavity
	return outer.MassRecord(), cavity.MassRecord()
}
