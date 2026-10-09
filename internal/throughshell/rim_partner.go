package throughshell

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RimTool identifies the holes and frame of one tool crossing a removed wall.
type RimTool struct {
	W0, W1       int
	Loop0, Loop1 int
	FrameEmbed   brepgeom.Embed
}

// RimPartnerInput contains the section records used to pair a cavity hole
// with a hole of one removed face.
type RimPartnerInput struct {
	FaceIndex int
	Wall      bool
	Holes     []sectionrecord.LoopRecord
	Embed     brepgeom.Embed
	Caps      brepgeom.PrismCaps
	Joined    momentinput.Profile
	Tools     []RimTool
	Dilated   []momentinput.Profile
}

// RimMatch returns a removed-face hole index, or an SG7 reason for its caller
// to attribute to the removed face.
type RimMatch func(inR, inF sectionrecord.LoopRecord) (int, string)

// NewRimPartner prepares cap or wall hole matching for one removed face.
// A nonempty reason identifies an SG7 refusal; other errors pass through.
func NewRimPartner(ctx context.Context, in RimPartnerInput) (RimMatch, string, error) {
	if !in.Wall {
		holes := make([]int, len(in.Holes))
		for hi := range holes {
			holes[hi] = hi
			if in.FaceIndex == in.Caps.Bottom {
				holes[hi] = in.Caps.BottomLoop[1+hi] - 1
			}
		}
		return func(_, inF sectionrecord.LoopRecord) (int, string) {
			si := slices.IndexFunc(in.Joined.Holes, func(want sectionrecord.LoopRecord) bool {
				return brepgeom.LoopsEqual(want, inF)
			})
			if si < 0 {
				return -1, "a cavity face in its plane holds a loop that is neither its outer loop nor a hole of the eroded section"
			}
			hi := slices.Index(holes, si)
			if hi < 0 {
				return -1, "a hole of the eroded section partners no hole of the removed face"
			}
			return hi, ""
		}, "", nil
	}
	type want struct {
		loop sectionrecord.LoopRecord
		hi   int
	}
	var wants []want
	for i, tool := range in.Tools {
		var hi int
		switch in.FaceIndex {
		case tool.W0:
			hi = tool.Loop0 - 1
		case tool.W1:
			hi = tool.Loop1 - 1
		default:
			continue
		}
		p := in.Dilated[i]
		section, _, err := brepgeom.JoinProfile(brepgeom.Profile{Outer: p.Outer, Holes: p.Holes})
		if err != nil {
			return nil, "", err
		}
		inR, ok := brepgeom.NewPrismMap(tool.FrameEmbed, in.Embed).Loop(section.Outer)
		if !ok {
			return nil, "a dilated tool's section does not map into its frame", nil
		}
		loop, err := offset2d.ReverseLoopRecordContext(ctx, inR)
		if err != nil {
			return nil, "", err
		}
		wants = append(wants, want{loop: loop, hi: hi})
	}
	return func(inR, _ sectionrecord.LoopRecord) (int, string) {
		for _, w := range wants {
			if LoopsSame(w.loop, inR) {
				return w.hi, ""
			}
		}
		return -1, "a cavity face in its plane holds a loop that is neither its outer loop nor the dilated section of a tool through it"
	}, "", nil
}
