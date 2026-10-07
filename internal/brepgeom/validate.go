package brepgeom

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// FaceRecord contains the fields the BRep record audit reads from one face.
type FaceRecord struct {
	Frame            r3.Frame
	Region           *Profile
	Wall             sectionrecord.CurveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Delta            float64
	Role             string
}

// ValidateFaces rejects malformed or unsupported BRep face records in input
// order. faceAt is called after the context check for its face index.
func ValidateFaces(ctx context.Context, count int, faceAt func(int) FaceRecord) error {
	if count == 0 {
		return fmt.Errorf(`%w: a brep payload holds no face`, decaderr.ErrDegenerate)
	}
	roles := make(map[string]struct{}, count)
	finite := func(values ...float64) bool {
		for _, v := range values {
			if math.IsNaN(v) || math.IsInf(v, 0) {
				return false
			}
		}
		return true
	}
	for fi := range count {
		if err := ctx.Err(); err != nil {
			return err
		}
		f := faceAt(fi)
		if !f.Frame.IsValid() {
			return fmt.Errorf(`%w: brep face %d has no valid frame`, decaderr.ErrDegenerate, fi)
		}
		planar := f.Region != nil
		if planar == (f.Wall != nil) {
			return fmt.Errorf(`%w: brep face %d must carry exactly one of a region and a wall`, decaderr.ErrDegenerate, fi)
		}
		if !finite(f.Z0, f.Z1, f.Z0Delta, f.Z1Delta, f.Delta) || f.Z0Delta < 0 || f.Z1Delta < 0 || f.Delta < 0 {
			return fmt.Errorf(`%w: brep face %d has a non-finite level or a negative displacement`, decaderr.ErrDegenerate, fi)
		}
		if f.Role == "" {
			return fmt.Errorf(`%w: brep face %d has no role`, decaderr.ErrDegenerate, fi)
		}
		if _, seen := roles[f.Role]; seen {
			return fmt.Errorf(`%w: brep role %q names two faces`, decaderr.ErrDegenerate, f.Role)
		}
		roles[f.Role] = struct{}{}
		var segs []sectionrecord.CurveSegment
		if planar {
			if f.Z0 != f.Z1 || f.Z0Delta != f.Z1Delta {
				return fmt.Errorf(`%w: planar brep face %d has two levels`, decaderr.ErrDegenerate, fi)
			}
			for _, loop := range append([]sectionrecord.LoopRecord{f.Region.Outer}, f.Region.Holes...) {
				if len(loop.Segments) == 0 {
					return fmt.Errorf(`%w: planar brep face %d has an empty loop`, decaderr.ErrDegenerate, fi)
				}
				segs = append(segs, loop.Segments...)
			}
		} else {
			if !(f.Z0 < f.Z1) {
				return fmt.Errorf(`%w: swept brep face %d has an empty interval`, decaderr.ErrDegenerate, fi)
			}
			segs = []sectionrecord.CurveSegment{f.Wall}
		}
		for _, seg := range segs {
			seg, err := sectionrecord.NormalizeSegment(seg)
			if err != nil {
				return err
			}
			switch seg.(type) {
			case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
			default:
				return fmt.Errorf(`%w: a brep face carries lines, circles and arcs only, not %T`, decaderr.ErrUnsupported, seg)
			}
		}
	}
	return nil
}
