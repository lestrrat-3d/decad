// Package compositesweep checks composite sweep limits and combines bounded
// readings from its already built analytic spans.
package compositesweep

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const (
	// MaxSpansPerCall bounds temporary span builds in one composite sweep.
	MaxSpansPerCall = 256
	maxFacesPerCall = 262_144
)

// Preflight bounds the span and face counts before any temporary body is built.
func Preflight(profile momentinput.Profile, spans int) error {
	if spans < 2 {
		return fmt.Errorf(`%w: a composite sweep requires at least two spans`, decaderr.ErrDegenerate)
	}
	if spans > MaxSpansPerCall {
		return fmt.Errorf(`%w: a composite sweep exceeds the fixed span ceiling of %d`, decaderr.ErrUnsupported, MaxSpansPerCall)
	}
	pairs, ok := proofbound.WallChoose2(uint64(spans))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the composite sweep audit exceeds its fixed pair budget`, decaderr.ErrUnsupported)
	}
	segments := len(profile.Outer.Segments)
	for _, hole := range profile.Holes {
		var ok bool
		segments, ok = checkedAdd(segments, len(hole.Segments))
		if !ok {
			return fmt.Errorf(`%w: a composite sweep's profile topology count overflows`, decaderr.ErrUnsupported)
		}
	}
	faces, ok := proofbound.WallCheckedMul(uint64(spans), uint64(segments))
	if !ok {
		return fmt.Errorf(`%w: a composite sweep's face count overflows`, decaderr.ErrUnsupported)
	}
	faces, ok = proofbound.WallCheckedAdd(faces, 2)
	if !ok || faces > maxFacesPerCall {
		return fmt.Errorf(`%w: a composite sweep exceeds the fixed face ceiling of %d`, decaderr.ErrUnsupported, maxFacesPerCall)
	}
	return nil
}

func checkedAdd(a, b int) (int, bool) {
	if b > int(^uint(0)>>1)-a {
		return 0, false
	}
	return a + b, true
}
