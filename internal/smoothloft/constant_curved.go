package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// HasCurvedSection reports whether the recorded outer walk contains a curved segment.
func HasCurvedSection(profile momentinput.Profile) bool {
	for _, segment := range profile.Outer.Segments {
		if _, ok := segment.(sectionrecord.LineSeg); !ok {
			return true
		}
	}
	return false
}

// ConstantCurvedFrameHeight admits three identical curved section records and
// returns the frame and exact height used by the analytic prism builder.
func ConstantCurvedFrameHeight(ctx context.Context, profiles [3]momentinput.Profile,
	planes [3]sectionrecord.PlaneRecord) (r3.Frame, float64, error) {
	if len(profiles[0].Holes) != 0 || len(profiles[0].Outer.Segments) < 3 ||
		len(profiles[0].Outer.Segments) > 64 {
		return r3.Frame{}, 0,
			fmt.Errorf("%w: a curved three-section loft needs one outer loop of 3 to 64 segments", decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	for i := 1; i < 3; i++ {
		same, err := momentinput.ExactProfileEqual(budget, profiles[0], profiles[i])
		if err != nil {
			return r3.Frame{}, 0, err
		}
		if !same {
			return r3.Frame{}, 0,
				fmt.Errorf("%w: curved three-section loft profiles must have identical records", decaderr.ErrUnsupported)
		}
	}
	for i := range planes {
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return r3.Frame{}, 0,
				fmt.Errorf("%w: curved three-section loft planes need common world-XY axes and origin", decaderr.ErrUnsupported)
		}
	}
	z0 := new(big.Rat).SetFloat64(planes[0].Origin.Z)
	z1 := new(big.Rat).SetFloat64(planes[1].Origin.Z)
	z2 := new(big.Rat).SetFloat64(planes[2].Origin.Z)
	if z0.Cmp(z1) >= 0 || z1.Cmp(z2) >= 0 ||
		new(big.Rat).Mul(big.NewRat(2, 1), z1).Cmp(new(big.Rat).Add(z0, z2)) != 0 {
		return r3.Frame{}, 0,
			fmt.Errorf("%w: curved three-section loft planes need ascending, equally spaced levels", decaderr.ErrUnsupported)
	}
	height, exact := new(big.Rat).Sub(z2, z0).Float64()
	if !exact || math.IsInf(height, 0) || height <= 0 {
		return r3.Frame{}, 0,
			fmt.Errorf("%w: curved three-section loft height must be held exactly", decaderr.ErrUnsupported)
	}
	frame, err := r3.NewFrame(planes[0].Origin, planes[0].U, planes[0].V)
	if err != nil {
		return r3.Frame{}, 0,
			fmt.Errorf("%w: curved three-section loft has no first frame: %s", decaderr.ErrDegenerate, err)
	}
	return frame, height, nil
}
