package thickenaxis

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
)

// SectionPair holds the certified outer and inner offsets of one sheet section.
type SectionPair struct {
	Outer, Inner momentinput.Profile
}

func NewSectionPair(source momentinput.Profile) SectionPair {
	return SectionPair{Outer: source, Inner: source}
}

// Generated selects the offset for a one-sided wall.
func (s SectionPair) Generated(negative bool) momentinput.Profile {
	if negative {
		return s.Inner
	}
	return s.Outer
}

// Sense names which side of the generated section the source occupies.
func (s SectionPair) Sense(negative bool) float64 {
	if negative {
		return +1
	}
	return -1
}

// Annulus reverses the inner loop into the hole of the outer section.
func (s SectionPair) Annulus(ctx context.Context) (momentinput.Profile, error) {
	hole, err := offset2d.ReverseLoopRecordContext(ctx, s.Inner.Outer)
	if err != nil {
		return momentinput.Profile{}, err
	}
	return momentinput.Profile{Outer: s.Outer.Outer, Holes: []momentinput.LoopRecord{hole}}, nil
}
