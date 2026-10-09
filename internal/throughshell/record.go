package throughshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// FaceRead is TC1–TC3's reading of the receiver's prism and tool wall candidates.
type FaceRead struct {
	Caps               brepgeom.PrismCaps
	Kinds              []brepgeom.ThroughFace
	ZloDelta, ZhiDelta float64
}

// ReadThroughFaces classifies the record's faces along one reference axis.
// A failed reading returns the first face-specific reason and no error.
func ReadThroughFaces(ctx context.Context, faces []brepgeom.PrismRectFace,
	embeds []brepgeom.Embed, axis int) (FaceRead, string, error) {
	caps, ok := brepgeom.ReadPrismCaps(faces, embeds, axis)
	if !ok {
		return FaceRead{}, capsReason(faces, embeds, axis), nil
	}
	read := FaceRead{Caps: caps, Kinds: make([]brepgeom.ThroughFace, len(faces))}
	read.Kinds[caps.Bottom], read.Kinds[caps.Top] = brepgeom.ThroughCap, brepgeom.ThroughCap
	var walls brepgeom.PrismWallSet
	work := freeform.NewFreeformWork()
	holeKeys, err := brepgeom.HoleWalkKeys(caps.Section, work)
	if err != nil {
		return FaceRead{}, "the section's hole walls have no walk", nil //nolint:nilerr // no walk is no reading
	}
	type level struct {
		axis  int
		value float64
	}
	pierced := map[level][]int{}
	var candidates []int
	for fi, f := range faces {
		if err := ctx.Err(); err != nil {
			return FaceRead{}, "", err
		}
		if fi == caps.Bottom || fi == caps.Top {
			continue
		}
		e := embeds[fi]
		switch {
		case e.Axis[2] == axis && f.Region == nil:
			if !walls.Add(f, e, caps.Embed, axis, caps.Zlo, caps.Zhi, work) {
				return FaceRead{}, faceReason(f, "is a wall along the axis that does not span the prism between its caps"), nil
			}
			read.Kinds[fi] = brepgeom.ThroughWall
			if seg, ok := brepgeom.NewPrismMap(e, caps.Embed).Segment(f.Wall); ok {
				if w, err := boundarywalk.WalkOf(seg, work); err == nil {
					if _, hole := holeKeys[brepgeom.WalkKeyOf(w)]; hole {
						read.Kinds[fi] = brepgeom.ThroughHoleWall
					}
				}
			}
		case f.Region != nil:
			outer := *f.Region
			outer.Holes = nil
			bare := f
			bare.Region = &outer
			if !walls.Add(bare, e, caps.Embed, axis, caps.Zlo, caps.Zhi, work) {
				return FaceRead{}, faceReason(f, "is a planar face across another axis whose outer loop is no wall rectangle of the prism"), nil
			}
			read.Kinds[fi] = brepgeom.ThroughPierced
			key := level{axis: e.Axis[2], value: e.Sign[2]*f.Z0 + 0}
			pierced[key] = append(pierced[key], fi)
		default:
			if f.Split0 || f.Split1 || f.Z0Delta != 0 || f.Z1Delta != 0 ||
				!classbgeom.NaturalRecord([]sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{f.Wall}}}) {
				return FaceRead{}, faceReason(f, "is a swept face across the axis that is no unsplit, undisplaced tool wall over its natural range"), nil
			}
			read.Kinds[fi] = brepgeom.ThroughTool
			candidates = append(candidates, fi)
		}
	}
	if !walls.ClaimsSection(caps.Section, work) {
		return FaceRead{}, "the walls of the prism do not claim each segment of its section exactly once", nil
	}
	read.ZloDelta, read.ZhiDelta = walls.ZloDelta, walls.ZhiDelta
	for _, fi := range candidates {
		f, e := faces[fi], embeds[fi]
		l0, l1 := e.Sign[2]*f.Z0+0, e.Sign[2]*f.Z1+0
		_, ok0 := pierced[level{axis: e.Axis[2], value: l0}]
		_, ok1 := pierced[level{axis: e.Axis[2], value: l1}]
		if !ok0 || !ok1 {
			return FaceRead{}, faceReason(f, "is a swept face across the axis whose levels are not two pierced walls' levels"), nil
		}
	}
	return read, "", nil
}

func faceReason(f brepgeom.PrismRectFace, what string) string {
	return fmt.Sprintf("%s %s", f.Role, what)
}

func capsReason(faces []brepgeom.PrismRectFace, embeds []brepgeom.Embed, axis int) string {
	var across []int
	for fi, f := range faces {
		if f.Region != nil && embeds[fi].Axis[2] == axis {
			across = append(across, fi)
		}
	}
	if len(across) > 2 {
		return faceReason(faces[across[2]], "is a third planar face across the axis")
	}
	return fmt.Sprintf("%d planar faces lie across the axis, and a prism has its two caps there, holding one section", len(across))
}
