package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// This file is route P of docs/brep-modify-design.md ("brep-modify §N"): a
// brep or stacked receiver that reads as a prism along a reference axis
// (§4.1, P1–P5) is handed to the op's own prism path as that prism (§4.2).
// Every test below is an exact comparison of recorded floats and records,
// moved between face frames by signed permutations; nothing is sampled,
// solved or admitted on a residual.

// prismCaps names the two cap faces a modify op classifies a prism selection
// against (brep-modify §4.2): start at the prism's z0 level, end at its z1
// level. startLoop and endLoop map a cap face's loop index to the section
// loop it bounds (0 the outer, 1+i hole i); nil reads as the identity. A nil
// face names no cap, and a selection on it classifies as no cap edge.
type prismCaps struct {
	start, end         *Face
	startLoop, endLoop []int
}

// prismCapsOf names a prism body's own capStart and capEnd faces.
func prismCapsOf(b *Body) prismCaps {
	roles := facesByRole(b)
	return prismCaps{start: roles[roleCapStart], end: roles[roleCapEnd]}
}

// sectionLoop maps one cap face's loop index to its section loop index.
func (c prismCaps) sectionLoop(start bool, li int) int {
	loops := c.endLoop
	if start {
		loops = c.startLoop
	}
	if loops == nil {
		return li
	}
	return loops[li]
}

// brepPrismRoute is brep-modify §4.2: axes 0, 1, 2 in order, the first whose
// prism reading (§4.1) the op's own classification admits is taken. The
// returned route carries that prism and its caps. When no axis admits, the
// route is empty and refusal is the first axis's classification refusal, or
// nil when no axis reads as a prism at all. err is a context error, or the
// refusal of a Shell of a record carrying route L chamfer bands.
//
// A record carrying route L bands (docs/modify-general-design.md §4) reads as
// no prism: a prism reading would read its faces alone and drop the band
// patches the body also holds, so a Fillet or Chamfer goes on to route E or
// route L over the record. A Shell of it refuses, since its erosion meets the
// band patches, which no through-cut record holds (modify-general SG3).
func brepPrismRoute(ctx context.Context, b *Body, bp brepPayload, req brepModifyRequest) (brepRoute, error, error) {
	if len(bp.loopBands) > 0 {
		if req.shell {
			return brepRoute{}, nil, fmt.Errorf(`%w: this evaluator shells no brep body carrying route L chamfer bands; their patches are oblique planes and cones no through-cut record holds (modify-general SG3)`, ErrUnsupported)
		}
		return brepRoute{}, nil, nil
	}
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		// A record the build took has embeds; one that has none reads as no
		// prism, and the caller's own refusal follows.
		return brepRoute{}, nil, nil //nolint:nilerr // no embeds is no prism reading
	}
	var refusal error
	for k := range 3 {
		read, ok, err := recognisePrism(ctx, bp, embeds, k)
		if err != nil {
			return brepRoute{}, nil, err
		}
		if !ok {
			continue
		}
		caps := read.caps(b, bp)
		admitErr := req.admits(read.pp, caps)
		if admitErr == nil {
			pp := read.pp
			return brepRoute{prism: &pp, caps: caps}, nil, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return brepRoute{}, nil, ctxErr
		}
		if errors.Is(admitErr, context.Canceled) || errors.Is(admitErr, context.DeadlineExceeded) {
			return brepRoute{}, nil, admitErr
		}
		if refusal == nil {
			refusal = admitErr
		}
	}
	return brepRoute{}, refusal, nil
}

// brepPrismRead is one prism reading of a brep record along a reference axis
// (brep-modify §4.1): the recognised prism, the indices of the record's
// bottom and top faces, and which section loop each of bottom's region loops
// is (top's region is the section, loop for loop).
type brepPrismRead struct {
	pp          prismPayload
	bottom, top int
	bottomLoop  []int
}

// caps names the reading's cap faces on the receiver body: bottom is the
// prism's start cap and top its end cap. A stacked receiver's faces carry the
// stacked body's roles, not its face view's, so it names no cap face.
func (r brepPrismRead) caps(b *Body, bp brepPayload) prismCaps {
	if _, ok := b.payload.(brepPayload); !ok {
		return prismCaps{}
	}
	roles := facesByRole(b)
	return prismCaps{
		start:     roles[bp.faces[r.bottom].role],
		end:       roles[bp.faces[r.top].role],
		startLoop: r.bottomLoop,
	}
}

// brepPrismWalls collects what P2 and P5 read from the record's other faces:
// every swept face along the axis as its wall re-expressed into F, every
// rectangle across it as its projection onto F's plane with the outward
// normal there, and the level displacements the walls state at zlo and zhi.
type brepPrismWalls struct {
	walls              []survey2d.SegmentWalk
	rects              []brepgeom.PrismRect
	zloDelta, zhiDelta float64
}

// brepPrismCaps is what P1, P3 and P4 read: the record's two cap faces across
// reference axis k, their levels, the prism frame F with its embed, the
// section S in F, and which section loop each of the bottom's region loops is.
type brepPrismCaps struct {
	bottom, top int
	zlo, zhi    float64
	frame       r3.Frame
	eF          brepEmbed
	section     ProfileRecord
	bottomLoop  []int
}

// readPrismCaps reads brep-modify §4.1's P1, P3 and P4 along reference axis
// k, or reports false. It reads no wall, so a caller that classifies the other
// faces itself (docs/modify-general-design.md Table TC) shares the cap reading
// with recognisePrism.
func readPrismCaps(bp brepPayload, embeds []brepEmbed, k int) (brepPrismCaps, bool) {
	// P1: exactly two planar faces across the axis, bottom facing −k and top
	// facing +k, with zlo < zhi.
	bottom, top := -1, -1
	planar := 0
	for fi, f := range bp.faces {
		if !f.planar() || embeds[fi].Axis[2] != k {
			continue
		}
		planar++
		if brepOutwardSign(f, embeds[fi]) < 0 {
			bottom = fi
		} else {
			top = fi
		}
	}
	if planar != 2 || bottom < 0 || top < 0 {
		return brepPrismCaps{}, false
	}
	zlo := brepLevel(bp.faces[bottom], embeds[bottom])
	zhi := brepLevel(bp.faces[top], embeds[top])
	if !(zlo < zhi) {
		return brepPrismCaps{}, false
	}

	// P3: the prism frame F.
	var frame r3.Frame
	var eF brepEmbed
	switch {
	case embeds[top].Sign[2] > 0:
		frame, eF = bp.faces[top].frame, embeds[top]
	case embeds[bottom].Sign[2] > 0:
		frame, eF = bp.faces[bottom].frame, embeds[bottom]
	default:
		var err error
		frame, eF, err = brepgeom.AxisFrame(bp.faces[0].frame, k)
		if err != nil {
			return brepPrismCaps{}, false // no exact F is no prism reading
		}
	}

	// P4: the section S is top's region in F; bottom's region in F equals it.
	section, ok := brepgeom.NewPrismMap(embeds[top], eF).Region(*bp.faces[top].region)
	if !ok {
		return brepPrismCaps{}, false
	}
	bottomRegion, ok := brepgeom.NewPrismMap(embeds[bottom], eF).Region(*bp.faces[bottom].region)
	if !ok {
		return brepPrismCaps{}, false
	}
	bottomLoop, ok := brepgeom.SameRegion(section, bottomRegion)
	if !ok {
		return brepPrismCaps{}, false
	}
	return brepPrismCaps{
		bottom: bottom, top: top, zlo: zlo, zhi: zhi,
		frame: frame, eF: eF, section: section, bottomLoop: bottomLoop,
	}, true
}

// classifyPrismWalls is brep-modify §4.1's P2 and P5 over the faces other than
// the caps: every other face is a wall along the axis or a rectangle across
// it, and each claims one segment of the section exactly once. It reports
// false for a record that fails either. The error is a context error only.
func classifyPrismWalls(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int, caps brepPrismCaps) (brepPrismWalls, bool, error) {
	// P2: every other face is a wall along the axis or a rectangle across it.
	var walls brepPrismWalls
	work := freeform.NewFreeformWork()
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return brepPrismWalls{}, false, err
		}
		if fi == caps.bottom || fi == caps.top {
			continue
		}
		if !walls.add(f, embeds[fi], caps.eF, k, caps.zlo, caps.zhi, work) {
			return brepPrismWalls{}, false, nil
		}
	}

	// P5: every wall and rectangle claims one segment of S, each exactly once.
	if !brepgeom.PrismWallsClaim(walls.walls, walls.rects, caps.section, work) {
		return brepPrismWalls{}, false, nil
	}
	return walls, true, nil
}

// recognisePrism reads the record as a prism along reference axis k, or
// reports false (brep-modify §4.1, P1–P5): readPrismCaps for P1, P3 and P4,
// then classifyPrismWalls for P2 and P5. The error is a context error only.
func recognisePrism(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int) (brepPrismRead, bool, error) {
	caps, ok := readPrismCaps(bp, embeds, k)
	if !ok {
		return brepPrismRead{}, false, nil
	}
	walls, ok, err := classifyPrismWalls(ctx, bp, embeds, k, caps)
	if err != nil || !ok {
		return brepPrismRead{}, false, err
	}
	return brepPrismRead{
		pp: prismPayload{
			profile: caps.section, frame: caps.frame, xform: bp.xform,
			z0: caps.zlo, z1: caps.zhi,
			z0Delta: max(bp.faces[caps.bottom].z0Delta, walls.zloDelta),
			z1Delta: max(bp.faces[caps.top].z0Delta, walls.zhiDelta),
		},
		bottom: caps.bottom, top: caps.top, bottomLoop: caps.bottomLoop,
	}, true, nil
}

// brepLevel is a planar face's level as a reference coordinate.
func brepLevel(f brepFace, e brepEmbed) float64 { return e.Sign[2]*f.z0 + 0 }

// brepOutwardSign is the sign of a planar face's outward normal along its
// reference axis.
func brepOutwardSign(f brepFace, e brepEmbed) float64 {
	if f.outward {
		return e.Sign[2]
	}
	return -e.Sign[2]
}

// add classifies one face against P2 and records what P5 reads from it. It
// reports false for a face P2 does not admit.
func (w *brepPrismWalls) add(f brepFace, e, eF brepEmbed, k int, zlo, zhi float64, work *freeform.FreeformWork) bool {
	record := brepgeom.PrismRectFace{
		Region: f.region, Wall: f.wall, Z0: f.z0, Z1: f.z1,
		Z0Delta: f.z0Delta, Z1Delta: f.z1Delta,
		Split0: len(f.side0) != 0, Split1: len(f.side1) != 0, Outward: f.outward,
	}
	switch {
	case !f.planar() && e.Axis[2] == k:
		// (a): a wall along the axis over exactly [zlo, zhi], unsplit.
		if len(f.side0) != 0 || len(f.side1) != 0 {
			return false
		}
		l0, l1 := e.Sign[2]*f.z0+0, e.Sign[2]*f.z1+0
		d0, d1 := f.z0Delta, f.z1Delta
		if l0 > l1 {
			l0, l1, d0, d1 = l1, l0, d1, d0
		}
		if l0 != zlo || l1 != zhi {
			return false
		}
		seg, ok := brepgeom.NewPrismMap(e, eF).Segment(f.wall)
		if !ok {
			return false
		}
		walk, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return false
		}
		w.walls = append(w.walls, walk)
		w.zloDelta, w.zhiDelta = max(w.zloDelta, d0), max(w.zhiDelta, d1)
		return true
	case f.planar():
		// (b): a rectangle across the axis spanning exactly [zlo, zhi].
		rect, ok := brepgeom.PlanarPrismRect(record, e, eF, k, zlo, zhi)
		if ok {
			w.rects = append(w.rects, rect)
		}
		return ok
	default:
		// (c): the same rectangle stated as a straight wall along the axis.
		rect, ok := brepgeom.SweptPrismRect(record, e, eF, k, zlo, zhi)
		if ok {
			w.rects = append(w.rects, rect)
		}
		return ok
	}
}

// requireLateralEdges is Fillet's route P classification (brep-modify §4.2):
// every selected edge is a lateral edge of pp, the junction at one corner of
// its section (matchCornerBudget). Any other edge is modify S1's class.
func requireLateralEdges(ctx context.Context, pp prismPayload, edges []*Edge) error {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := prismCornerLoopsBudget(budget, pp)
	if err != nil {
		return err
	}
	for ei, e := range edges {
		_, _, found, err := matchCornerBudget(budget, pp, loops, e)
		if err != nil {
			return err
		}
		if !found {
			return fmt.Errorf(`%w: %s is not a lateral edge of the prism`, ErrUnsupported, selectedEdgeContext(ei, e))
		}
	}
	return nil
}
