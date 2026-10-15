package breprecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// Topology is the shared reading of a record that the body build and the
// tessellator both take: each face's walks, every edge use, and how the uses
// pair into edges.
type Topology struct {
	Embeds []brepgeom.Embed
	// walls holds a swept face's walk, planar holds each planar face's walks
	// per region loop.
	Walls  map[int]survey2d.SegmentWalk
	Planar map[int][][]survey2d.SegmentWalk
	Uses   []brepgeom.Use
	// edges lists the two use indices of every edge, the owner first: a rim use
	// where the edge has one, then a side line, then a loop segment. The
	// owner's natural direction is the edge's direction.
	Edges [][2]int
	// edgeOf maps a use index to its edge index, -1 for an open use.
	EdgeOf []int
	// open lists the uses on a route L band's boundary
	// (docs/modify-general-design.md §4.2): the cap contour's segments on the
	// band's face and the side contour's on the faces beside it. The record
	// bounds each such edge on one side only, and the band's patches bound
	// it on the other (band.go).
	Open []int
	// faceUses lists each face's use indices in loop order: a swept face's
	// rim0, side1, rim1, side0 (a whole circle's rim0 and rim1 alone); a
	// planar face's loops in record order.
	FaceUses [][]int
	// coordUpper is the largest |u|+|v| over every walk and |z| over every
	// level: the input magnitude the frame and placement lift rounds against.
	CoordUpper float64
	// regions caches each planar face's integrated region (Topology.region).
	Regions map[int]brepgeom.Region
}

// TopologyContext walks every face, keys every edge use, and pairs them.
// An edge that does not bound exactly two distinct faces refuses: the build
// proves closure by counting (§4.2), so an unpaired use is a record this
// evaluator cannot close and is decaderr.ErrUnsupported. The one exception is a
// route L band's boundary (Payload.loopBandKeys): each such edge must
// bound exactly one face of the record, and its use is listed in open.
func TopologyContext(ctx context.Context, bp Payload) (*Topology, error) {
	if err := Falsify(ctx, bp); err != nil {
		return nil, err
	}
	embeds, err := Embeds(bp.Faces)
	if err != nil {
		return nil, err
	}
	faces := make([]brepgeom.FaceWalks, len(bp.Faces))
	work := freeform.NewFreeformWork()
	walk := func(seg sectionrecord.CurveSegment) (survey2d.SegmentWalk, error) {
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, "a brep face"); err != nil {
			return survey2d.SegmentWalk{}, err
		}
		return w, nil
	}
	for fi, f := range bp.Faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face := brepgeom.FaceWalks{Embed: embeds[fi], IsPlanar: f.Planar(),
			Outward: f.Outward, Sweep: brepgeom.NoSweep,
			Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
			Side0: f.Side0, Side1: f.Side1}
		if f.Planar() {
			if f.Sweep != (r3.Vec{}) {
				axis, ok := brepgeom.SweepAxis(bp.Faces[0].Frame, f.Sweep)
				if !ok || axis == embeds[fi].Axis[2] {
					return nil, fmt.Errorf(`%w: brep face %d's recorded sweep is not a reference axis in its plane`, decaderr.ErrUnsupported, fi)
				}
				face.Sweep = axis
			}
			loops := append([]sectionrecord.LoopRecord{f.Region.Outer}, f.Region.Holes...)
			face.Planar = make([][]survey2d.SegmentWalk, len(loops))
			face.PlanarSegs = make([][]sectionrecord.CurveSegment, len(loops))
			for li, loop := range loops {
				face.PlanarSegs[li] = loop.Segments
				for _, seg := range loop.Segments {
					w, err := walk(seg)
					if err != nil {
						return nil, err
					}
					face.Planar[li] = append(face.Planar[li], w)
				}
			}
			faces[fi] = face
			continue
		}
		w, err := walk(f.Wall)
		if err != nil {
			return nil, err
		}
		face.Wall, face.WallSeg = w, f.Wall
		faces[fi] = face
	}
	open, err := bp.LoopBandKeys(embeds, walk)
	if err != nil {
		return nil, err
	}
	built, err := brepgeom.Build(faces, open, decaderr.ErrUnsupported)
	if err != nil {
		return nil, err
	}
	return &Topology{Embeds: embeds, Walls: built.Walls, Planar: built.Planar,
		Uses: built.Uses, Edges: built.Edges, EdgeOf: built.EdgeOf, Open: built.Open,
		FaceUses: built.FaceUses, CoordUpper: built.CoordUpper}, nil
}

// forward reports whether use ui walks its edge in the edge's own direction,
// which is its owner use's natural one. A whole circle compares senses: it has
// no endpoints to compare.
func (btopo *Topology) Forward(ui int) bool {
	return brepgeom.Forward(btopo.Uses, btopo.Edges, btopo.EdgeOf, ui)
}

// region reads a planar face's region once per topology.
func (btopo *Topology) Region(ctx context.Context, bp Payload, fi int) (brepgeom.Region, error) {
	if r, ok := btopo.Regions[fi]; ok {
		return r, nil
	}
	f := bp.Faces[fi]
	r, err := brepgeom.RegionOf(ctx, f.Region, f.Delta, btopo.Planar[fi])
	if err != nil {
		return brepgeom.Region{}, err
	}
	if btopo.Regions == nil {
		btopo.Regions = map[int]brepgeom.Region{}
	}
	btopo.Regions[fi] = r
	return r, nil
}
