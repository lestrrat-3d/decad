package throughshell

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RimTraceInput names the cavity faces and the removed face's plane. Partner
// identifies the removed-face hole paired with each cavity hole.
type RimTraceInput struct {
	Faces     []brepgeom.PrismRectFace
	Embeds    []brepgeom.Embed
	Holes     []sectionrecord.LoopRecord
	Embed     brepgeom.Embed
	Axis      int
	Level     float64
	Wall      bool
	SweepAxis int
	CapsEmbed brepgeom.Embed
	Partner   RimMatch
}

// RimTrace records the cavity trace in the removed face's plane and the band
// between each pair of cavity and removed-face holes.
type RimTrace struct {
	Outers  []sectionrecord.LoopRecord
	Bands   []momentinput.Profile
	InPlane []int
}

// TraceRimFaces reads every cavity face in the removed face's plane. A
// nonempty reason is an SG7 refusal that the caller attributes to that face.
func TraceRimFaces(ctx context.Context, in RimTraceInput) (RimTrace, string, error) {
	var trace RimTrace
	partnered := map[int]struct{}{}
	for ci, q := range in.Faces {
		eQ := in.Embeds[ci]
		if q.Region == nil {
			if !in.Wall || eQ.Axis[2] != in.SweepAxis {
				continue
			}
			outer, ok := SweptTrace(q.Wall, q.Z0, q.Z1, eQ, in.Embed, in.Axis, in.Level, in.SweepAxis)
			if !ok {
				continue
			}
			trace.InPlane = append(trace.InPlane, ci)
			trace.Outers = append(trace.Outers, outer)
			continue
		}
		if eQ.Axis[2] != in.Axis || eQ.Sign[2]*q.Z0+0 != in.Level {
			continue
		}
		trace.InPlane = append(trace.InPlane, ci)
		inR, ok := brepgeom.NewPrismMap(eQ, in.Embed).Region(*q.Region)
		if !ok {
			return RimTrace{}, "a cavity face in its plane does not map into its frame", nil
		}
		inF, ok := brepgeom.NewPrismMap(eQ, in.CapsEmbed).Region(*q.Region)
		if !ok && !in.Wall {
			return RimTrace{}, "a cavity face in its plane does not map into the prism's frame", nil
		}
		trace.Outers = append(trace.Outers, inR.Outer)
		for qh, hole := range inR.Holes {
			var holeF sectionrecord.LoopRecord
			if !in.Wall {
				holeF = inF.Holes[qh]
			}
			hi, reason := in.Partner(hole, holeF)
			if reason != "" {
				return RimTrace{}, reason, nil
			}
			if _, dup := partnered[hi]; dup {
				return RimTrace{}, "a cavity hole partners no hole of the removed face, or two", nil
			}
			partnered[hi] = struct{}{}
			band, err := offset2d.ReverseLoopRecordContext(ctx, hole)
			if err != nil {
				return RimTrace{}, "", err
			}
			trace.Bands = append(trace.Bands, momentinput.Profile{
				Outer: band, Holes: []sectionrecord.LoopRecord{in.Holes[hi]},
			})
		}
	}
	if len(partnered) != len(in.Holes) {
		return RimTrace{}, "a hole of the removed face has no cavity hole to partner", nil
	}
	return trace, "", nil
}
