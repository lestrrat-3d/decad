package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
			return brepRoute{}, nil, fmt.Errorf(`%w: this evaluator shells no brep body carrying route L loop bands; their patches are oblique planes, cones, cylinders and tori no through-cut record holds (modify-general SG3)`, ErrUnsupported)
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

func prismFaceRecord(f brepFace) brepgeom.PrismRectFace {
	return brepgeom.PrismRectFace{
		Frame: f.frame, Region: f.region, Wall: f.wall, Z0: f.z0, Z1: f.z1,
		Z0Delta: f.z0Delta, Z1Delta: f.z1Delta, Split0: len(f.side0) != 0,
		Split1: len(f.side1) != 0, Outward: f.outward, Role: f.role,
	}
}

func prismFaceRecords(bp brepPayload) []brepgeom.PrismRectFace {
	faces := make([]brepgeom.PrismRectFace, len(bp.faces))
	for i, f := range bp.faces {
		faces[i] = prismFaceRecord(f)
	}
	return faces
}

// recognisePrism reads the record as a prism along reference axis k, or
// reports false (brep-modify §4.1, P1–P5): brepgeom.ReadPrismCaps for P1, P3 and P4,
// then brepgeom.ClassifyPrismWalls for P2 and P5. The error is a context error only.
func recognisePrism(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int) (brepPrismRead, bool, error) {
	faces := prismFaceRecords(bp)
	caps, ok := brepgeom.ReadPrismCaps(faces, embeds, k)
	if !ok {
		return brepPrismRead{}, false, nil
	}
	walls, ok, err := brepgeom.ClassifyPrismWalls(ctx, faces, embeds, k, caps)
	if err != nil || !ok {
		return brepPrismRead{}, false, err
	}
	return brepPrismRead{
		pp: prismPayload{
			profile: caps.Section, frame: caps.Frame, xform: bp.xform,
			z0: caps.Zlo, z1: caps.Zhi,
			z0Delta: max(bp.faces[caps.Bottom].z0Delta, walls.ZloDelta),
			z1Delta: max(bp.faces[caps.Top].z0Delta, walls.ZhiDelta),
		},
		bottom: caps.Bottom, top: caps.Top, bottomLoop: caps.BottomLoop,
	}, true, nil
}

// brepLevel is a planar face's level as a reference coordinate.
func brepLevel(f brepFace, e brepEmbed) float64 { return e.Sign[2]*f.z0 + 0 }

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
