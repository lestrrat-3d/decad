package decad

import (
	"context"
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is the cup of docs/modify-design.md §9 (Table B, B5/B6). A cup
// is the outer region O on its interval less the cavity region C on its own,
// sharing a rim at the open end and a floor at the closed one. Its record is a
// two-slab stackedPrismPayload (docs/modify-reach-design.md §9.1): a floor slab
// over O and a wall slab over the bands between O's loops and C's, with C
// exposed between them as the pocket floor. The stacked build makes its body,
// and every cup survey, mesh and mass reading reads the cupView the record
// re-derives. It re-evaluates under Transform (evaluator §8).
//
// A holed section (k ≥ 1 posts in the pocket) builds too: the outer region and
// the cavity region each carry k holes, so the wall slab holds 1 + k bands, one
// wrapping each post. The cavity is a VOID the solid encloses, so its loops
// invert — the void's outer boundary is a hole in the solid, each of the
// void's own holes a solid post — and every band still hangs off the one floor
// slab, so the result is one lump (Table B, B5/B6).

const roleShellCap = "shellCap"

// cupView is a cup read as two prisms, re-derived from its stacked record
// (cupPayload.view) for every cup survey, mesh and mass reading, and the form
// the shell assembles before recording it (cupView.payload): the outer region
// O and the cavity region C (each walked in its natural sense — the outer loop
// counter-clockwise), the plane frame, the three sweep planes, the private
// shell-morphology certificate and the accumulated rigid placement. The open
// end (the removed cap, where the rim is) is zOpen; the outer prism's floor is
// at zOuter, the cavity's floor (shellCap) at zCav, which lies between the two
// so the floor slab is [zOuter, zCav] (docs/modify-design.md §9).
//
// zOpenDelta, zOuterDelta and zCavDelta are the axial displacement of their
// matching levels. zOpen and the unstepped floor level retain the displacement
// of the receiver end they came from. A derived floor level additionally carries
// the thickness conversion and float-sum rounding. Every level-derived cup
// reading uses its matching displacement — the two heights and the volume, area
// and centroid built on them, the box, and the wall vertices and vertical edges
// the shared prism build stamps.
//
// thicknessDelta is the shell thickness's OWN conversion displacement — the
// bound extent.MagnitudeInBounded proved when it converted the caller's stated
// thickness into millimetres (shell.go). It is distinct from zOuterDelta and
// zCavDelta: those cover the derived FLOOR LEVEL's own float-sum rounding
// (which already folds thicknessDelta in — cupPayloadFor's step closure), and
// this one is the reading cupWall publishes when the shell-wall theorem holds
// exactly on the payload's own morphology — the number the theorem's t is
// held in, not the theorem itself (docs/payload-verification-design.md §4.1).
//
// offsetDelta is the OFFSET region's section displacement — the cavity
// inward, the outer region outward: the proven upper bound offsetSectionDelta
// (shell_offset.go) states on how far any recorded boundary point of that
// region sits from the boundary of P ⊖ t* or P ⊕ t* the shell denotes, t* the
// caller's thickness in exact millimetres. It covers the thickness conversion
// and the offset solve's own rounding together. The other region is the
// receiver's own section and carries none. outerPrism and cavityPrism hand it
// to the offset region's prism as its sectionDelta, so every view reading
// built on that region — Bounds, the mass path, the mesh — charges it there,
// and cupPlan charges it to the offset region's columns of the stacked build
// (docs/modify-design.md §9, §10).
type cupView struct {
	outer          profileRecord
	cavity         profileRecord
	frame          r3.Frame
	zOpen          float64
	zOuter         float64
	zCav           float64
	zOpenDelta     float64
	zOuterDelta    float64
	zCavDelta      float64
	thickness      float64
	thicknessDelta float64
	offsetDelta    float64
	sense          ShellSense
	xform          r3.Transform
}

// AxialDelta reports the largest held-level displacement to a body-relative
// stop resolved against this cup's faces (stops.go).
func (cp cupView) AxialDelta() float64 {
	return math.Max(cp.zOpenDelta, math.Max(cp.zOuterDelta, cp.zCavDelta))
}

// openScalar, outerScalar and cavityScalar are the cup's three sweep levels as
// bounded readings, each carrying its own axial displacement.
func (cp cupView) openScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.zOpen, cp.zOpenDelta)
}

func (cp cupView) outerScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.zOuter, cp.zOuterDelta)
}

func (cp cupView) cavityScalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(cp.zCav, cp.zCavDelta)
}

// basePrism shares the cup's frame and placement with the prism helpers that
// need no axial level.
func (cp cupView) basePrism() prismPayload {
	return prismrecord.Payload{Frame: cp.frame, Xform: cp.xform}
}

// prismBetween builds an ordered prism from two cup levels, preserving each
// level's own axial displacement at the matching prism end.
func (cp cupView) prismBetween(a, b proofbound.BoundedScalar) prismPayload {
	if a.Value <= b.Value {
		return prismrecord.Payload{
			Frame: cp.frame, Z0: a.Value, Z1: b.Value,
			Z0Delta: a.Bound, Z1Delta: b.Bound,
			Xform: cp.xform,
		}
	}
	return prismrecord.Payload{
		Frame: cp.frame, Z0: b.Value, Z1: a.Value,
		Z0Delta: b.Bound, Z1Delta: a.Bound,
		Xform: cp.xform,
	}
}

// outerPrism and cavityPrism are the cup's two prisms, without their
// profiles. The offset region's prism carries offsetDelta as its section
// displacement: the outer one outward, the cavity inward.
func (cp cupView) outerPrism() prismPayload {
	pp := cp.prismBetween(cp.outerScalar(), cp.openScalar())
	if cp.sense == Outward {
		pp.SectionDelta = cp.offsetDelta
	}
	return pp
}

func (cp cupView) cavityPrism() prismPayload {
	pp := cp.prismBetween(cp.cavityScalar(), cp.openScalar())
	if cp.sense != Outward {
		pp.SectionDelta = cp.offsetDelta
	}
	return pp
}

// ExtentAlong is the cup's extent interval along an arbitrary world direction
// g, beside the displacement its ends carry — the OUTER prism's reading
// (docs/modify-design.md §10, Table D, D5). The cavity is interior and reaches
// no farther than the outer region, so the outward extent is the solid outer
// prism's, read by the same prismPayload.ExtentAlong the outer prism's bounds
// already use. This is what a through-all stop consults when a cup is a live
// body in the sweep's path. An outward cup's outer region is the offset one:
// its recorded extent is read, and its offsetDelta joins the displacement the
// ends carry — moving every boundary point by at most offsetDelta moves an
// extreme along g by at most offsetDelta·|g|, and |g| ≤ its L1 norm.
func (cp cupView) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	outer := cp.outerPrism()
	outer.Profile = cp.outer
	displaced := outer.SectionDelta
	outer.SectionDelta = 0
	lo, hi, delta, err := outer.ExtentAlong(g)
	if err != nil || displaced == 0 {
		return lo, hi, delta, err
	}
	return lo, hi, proofbound.AbsSumUpper(delta, proofbound.ProductUpper(displaced, stackedrecord.VecL1(g))), nil
}

// cupPayloadFor assembles the cup record from the receiver prism, its offset
// section with that section's proven displacement, and the shell
// sense/opening (docs/modify-design.md §9). removedEnd
// opens the cup at the top (z1); a removed start opens it at the bottom (z0),
// its mirror. Inward, O is the original section P and C the erosion Q; outward,
// O is the dilation Q and C the original P.
func cupPayloadFor(pp prismPayload, offset profileRecord, s, t, tDelta, offsetDelta float64, removedEnd bool) cupView {
	z0, z1 := pp.Z0, pp.Z1
	o, c := pp.Profile, offset
	if s < 0 {
		o, c = offset, pp.Profile
	}
	sense := Inward
	if s < 0 {
		sense = Outward
	}
	cp := cupView{
		outer:          o,
		cavity:         c,
		frame:          pp.Frame,
		thickness:      t,
		thicknessDelta: tDelta,
		offsetDelta:    offsetDelta,
		sense:          sense,
		xform:          pp.Xform,
	}
	// step is the one derived floor level. It carries the source end's own
	// displacement, the thickness conversion, and this float sum's rounding.
	step := func(from, delta, by float64) (float64, float64) {
		to := from + by
		return to, proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))
	}
	if removedEnd { // open at the top
		cp.zOpen = z1
		cp.zOpenDelta = pp.Z1Delta
		if s > 0 {
			cp.zOuter, cp.zOuterDelta = z0, pp.Z0Delta
			cp.zCav, cp.zCavDelta = step(z0, pp.Z0Delta, t)
		} else {
			cp.zOuter, cp.zOuterDelta = step(z0, pp.Z0Delta, -t)
			cp.zCav, cp.zCavDelta = z0, pp.Z0Delta
		}
	} else { // open at the bottom — the mirror
		cp.zOpen = z0
		cp.zOpenDelta = pp.Z0Delta
		if s > 0 {
			cp.zOuter, cp.zOuterDelta = z1, pp.Z1Delta
			cp.zCav, cp.zCavDelta = step(z1, pp.Z1Delta, -t)
		} else {
			cp.zOuter, cp.zOuterDelta = step(z1, pp.Z1Delta, t)
			cp.zCav, cp.zCavDelta = z1, pp.Z1Delta
		}
	}
	return cp
}

// cupPayload is the cup's stored record (docs/modify-reach-design.md §9.1): a
// two-slab stackedPrismPayload plus the shell morphology certificate the cup's
// own surveys read (docs/payload-verification-design.md §4.1). The floor slab
// holds the outer region O over the floor interval; the wall slab holds the
// band between O's outer loop and the cavity's, then one band lining each
// hole, over the cavity interval; their interface exposes the cavity region C
// as the pocket floor. Every consumer that reads a cup reads its view, which
// re-derives the cup's regions and levels from the slabs.
//
// The morphology is the shell's own: thickness and thicknessDelta, the offset
// region's proven displacement offsetDelta, and the sense. offsetDelta is
// charged loop by loop (cupPlan), to the offset region's walls and to the
// planar faces those walls bound, never to the receiver's own loops.
type cupPayload struct {
	stack          stackedPrismPayload
	thickness      float64
	thicknessDelta float64
	offsetDelta    float64
	sense          ShellSense
}

// Transform is the accumulated rigid placement.
func (cp cupPayload) Transform() r3.Transform { return cp.stack.Xform }

// Placed re-evaluates the same cup under the composed motion (evaluator §8).
func (cp cupPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	cp.stack.Xform = composed
	return placedNode(evalCupContext(ctx, d, ref, cp))
}

// AxialDelta reports the largest held-level displacement (stops.go).
func (cp cupPayload) AxialDelta() float64 { return cp.stack.AxialDelta() }

// ExtentAlong is the view's outer-prism extent (cupView.ExtentAlong).
func (cp cupPayload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	return cp.view().ExtentAlong(g)
}

// floorSlab is the index of the slab holding the outer region: the first
// when the cup opens at the top, the second when it opens at the bottom.
func (cp cupPayload) floorSlab() int {
	if len(cp.stack.Slabs[0].Regions) == 1 && len(cp.stack.Slabs[1].Regions) == 1 {
		// A hole-free cup: the floor is the slab whose region has no hole.
		if len(cp.stack.Slabs[0].Regions[0].Holes) == 0 {
			return 0
		}
		return 1
	}
	if len(cp.stack.Slabs[0].Regions) == 1 {
		return 0
	}
	return 1
}

// view re-derives the cup's regions and levels from its slabs: the outer
// region is the floor slab's one region, the cavity is the interface's one
// exposed record, the floor slab's far end is zOuter, the interface is zCav,
// and the wall slab's far end is zOpen.
func (cp cupPayload) view() cupView {
	f := cp.floorSlab()
	floor, wall := cp.stack.Slabs[f], cp.stack.Slabs[1-f]
	boundary := cp.stack.Interfaces[0]
	v := cupView{
		outer:          floor.Regions[0],
		frame:          cp.stack.Frame,
		thickness:      cp.thickness,
		thicknessDelta: cp.thicknessDelta,
		offsetDelta:    cp.offsetDelta,
		sense:          cp.sense,
		xform:          cp.stack.Xform,
	}
	if f == 0 {
		v.cavity = boundary.LowerExposed[0]
		v.zOuter, v.zOuterDelta = floor.Z0, floor.Z0Delta
		v.zCav, v.zCavDelta = floor.Z1, floor.Z1Delta
		v.zOpen, v.zOpenDelta = wall.Z1, wall.Z1Delta
	} else {
		v.cavity = boundary.UpperExposed[0]
		v.zOuter, v.zOuterDelta = floor.Z1, floor.Z1Delta
		v.zCav, v.zCavDelta = floor.Z0, floor.Z0Delta
		v.zOpen, v.zOpenDelta = wall.Z0, wall.Z0Delta
	}
	return v
}

// payload records the view as a stacked cup: the floor slab over O, the wall
// slab over the band between O's outer and C's (C's outer reversed as that
// band's one hole) and, per hole i of O, the band between C's hole i
// (reversed, an outer) and O's hole i, and the interface exposing C itself.
// The two regions must have one loop count; a dropped loop is S11a, caught
// before here.
func (cp cupView) payload(ctx context.Context) (cupPayload, error) {
	bands, err := prismshell.ShellWallBands(ctx, cp.outer, cp.cavity)
	if err != nil {
		return cupPayload{}, err
	}
	floor := stackedrecord.Slab{Regions: []profileRecord{cp.outer}}
	wall := stackedrecord.Slab{Regions: bands}
	exposed := []profileRecord{cp.cavity}
	out := cupPayload{
		stack:          stackedPrismPayload{Frame: cp.frame, Xform: cp.xform},
		thickness:      cp.thickness,
		thicknessDelta: cp.thicknessDelta,
		offsetDelta:    cp.offsetDelta,
		sense:          cp.sense,
	}
	if cp.zOpen > cp.zOuter { // open at the top
		floor.Z0, floor.Z0Delta, floor.Z1, floor.Z1Delta = cp.zOuter, cp.zOuterDelta, cp.zCav, cp.zCavDelta
		wall.Z0, wall.Z0Delta, wall.Z1, wall.Z1Delta = cp.zCav, cp.zCavDelta, cp.zOpen, cp.zOpenDelta
		out.stack.Slabs = []stackedrecord.Slab{floor, wall}
		out.stack.Interfaces = []stackedrecord.Interface{{LowerExposed: exposed}}
	} else {
		wall.Z0, wall.Z0Delta, wall.Z1, wall.Z1Delta = cp.zOpen, cp.zOpenDelta, cp.zCav, cp.zCavDelta
		floor.Z0, floor.Z0Delta, floor.Z1, floor.Z1Delta = cp.zCav, cp.zCavDelta, cp.zOuter, cp.zOuterDelta
		out.stack.Slabs = []stackedrecord.Slab{wall, floor}
		out.stack.Interfaces = []stackedrecord.Interface{{UpperExposed: exposed}}
	}
	return out, nil
}

// cupPlan builds a cup's stack under the cup's own roles (Table B, B5/B6):
// outer walls side(i,j) indexing loop i of O, cavity walls shellSide(i,j)
// indexing loop i of C, the kept cap capStart, the pocket floor shellCap and
// the rims rim(i), one per wall band. A column crossing the interface is a
// loop of O; a column of the wall slab alone is a loop of C. The offset
// region's columns — C's inward, O's outward — carry offsetDelta.
type cupPlan struct {
	floor       int
	sense       ShellSense
	offsetDelta float64
}

// WallRoleLoop is the column's loop index in O, or in C for a cavity column.
// A wall-slab region m >= 1 lines O's hole m-1 with C's hole m-1, so loop m
// of either region; region 0 holds both outers, loop 0 of each.
func (p cupPlan) WallRoleLoop(col stackedrecord.WallColumn) int {
	if col.Start != col.End && col.Start == p.floor {
		return col.LoopIndex
	}
	return col.Region
}

func (p cupPlan) NameWalls(ctx context.Context, col stackedrecord.WallColumn, faces []*topo.Face, ref producerID) error {
	if col.Start != col.End {
		return nil
	}
	return renameCavityRoles(ctx, faces, ref)
}

func (p cupPlan) CapRole(slab, region int, _ bool) string {
	if slab == p.floor {
		return provenance.RoleCapStart
	}
	return fmt.Sprintf("rim(%d)", region)
}

func (cupPlan) PatchRole(stackedrecord.Patch) string { return roleShellCap }

func (p cupPlan) ColumnDelta(col stackedrecord.WallColumn) float64 {
	if (col.Start == col.End) == (p.sense != Outward) {
		return p.offsetDelta
	}
	return 0
}

// Order lists the faces as the cup always has: outer walls, cavity walls, the
// kept cap, the pocket floor, then the rims, each group in build Order.
func (cupPlan) Order(faces []*topo.Face) []*topo.Face {
	rank := func(f *topo.Face) int {
		role := f.Origins[0].Role
		switch {
		case strings.HasPrefix(role, "side("):
			return 0
		case strings.HasPrefix(role, "shellSide("):
			return 1
		case role == provenance.RoleCapStart:
			return 2
		case role == roleShellCap:
			return 3
		default:
			return 4
		}
	}
	out := slices.Clone(faces)
	slices.SortStableFunc(out, func(a, b *topo.Face) int { return rank(a) - rank(b) })
	return out
}

// evalCupContext builds the cup's stack through the stacked build (§9.1) under
// cupPlan, and keeps the cup record as the body's payload.
func evalCupContext(ctx context.Context, d *Document, ref producerID, cp cupPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body, err := bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), ref), cp.stack,
		cupPlan{floor: cp.floorSlab(), sense: cp.sense, offsetDelta: cp.offsetDelta}))
	if err != nil {
		return nil, err
	}
	body.n.Payload = cp
	return body, nil
}

// renameCavityRoles rewrites the cavity walls' provenance from the
// prismrecord.BuildLoopSidesAs default side(i,j) to shellSide(i,j) — the role Table B
// (B5/B6) gives a cavity wall, indexing loop i of the cavity region Q in the
// result's own record (§11). The cavity walls are built with roleLoop equal to
// their Q loop index, so the rename keeps the index and only swaps the tag.
func renameCavityRoles(ctx context.Context, faces []*topo.Face, ref producerID) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for i, o := range f.Origins {
			if err := ctx.Err(); err != nil {
				return err
			}
			if o.Producer != ref {
				continue
			}
			var li, j int
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n == 2 {
				f.Origins[i] = provenance.Ref{Producer: ref, Role: fmt.Sprintf("shellSide(%d,%d)", li, j)}
			}
		}
	}
	return nil
}
