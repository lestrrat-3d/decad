package proof

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// Replay refusals are constructed before any operation budget starts.
var (
	errReplayBoundedGcdOperands                 = fmt.Errorf("%w: bounded gcd operands", decaderr.ErrUnsupported)
	errReplayBoundedGcdWidth                    = fmt.Errorf("%w: bounded gcd width", decaderr.ErrUnsupported)
	errReplayBoundedDivisionOperands            = fmt.Errorf("%w: bounded division operands", decaderr.ErrUnsupported)
	errReplayBoundedDivisionWidth               = fmt.Errorf("%w: bounded division width", decaderr.ErrUnsupported)
	errReplayBoundedDivisionHasRemainder        = fmt.Errorf("%w: bounded division has remainder", decaderr.ErrUnsupported)
	errReplayBoundedDyadicWidthOverflow         = fmt.Errorf("%w: bounded dyadic width overflow", decaderr.ErrUnsupported)
	errReplayBoundedSquareRootNeedsAnOperandAnd = fmt.Errorf("%w: bounded square root needs an operand and budget", decaderr.ErrUnsupported)
	errReplayReplaySquareRootIsNotDyadic        = fmt.Errorf("%w: replay square root is not dyadic", decaderr.ErrUnsupported)
	errReplayInvalidDeterminantDenominator      = fmt.Errorf("%w: invalid determinant denominator", decaderr.ErrUnsupported)
	errReplayNonDyadicDeterminant               = fmt.Errorf("%w: non-dyadic determinant", decaderr.ErrUnsupported)
	errReplayReplayDeterminantInputWidth        = fmt.Errorf("%w: replay determinant input width", decaderr.ErrUnsupported)
	errReplayNonDyadicDeterminantResult         = fmt.Errorf("%w: non-dyadic determinant result", decaderr.ErrUnsupported)
	errReplayMissingOrientationBudget           = fmt.Errorf("%w: missing orientation budget", decaderr.ErrUnsupported)
	errReplayOrientationAllocationOverflow      = fmt.Errorf("%w: orientation allocation overflow", decaderr.ErrUnsupported)
	errReplayInvalidOrientationInputs           = fmt.Errorf("%w: invalid orientation inputs", decaderr.ErrUnsupported)
	errReplayReplayAffineWidth                  = fmt.Errorf("%w: replay affine width", decaderr.ErrUnsupported)
	errReplayReplayAffineDenominator            = fmt.Errorf("%w: replay affine denominator", decaderr.ErrUnsupported)
	errReplayExactNormalNeedsSharedCounters     = fmt.Errorf("%w: exact normal needs shared counters", decaderr.ErrUnsupported)
	errReplayExactNormalNeedsFiniteCoordinates  = fmt.Errorf("%w: exact normal needs finite coordinates", decaderr.ErrUnsupported)
	errReplayInvalidExactProduct                = fmt.Errorf("%w: invalid exact product", decaderr.ErrUnsupported)
	errReplayInvalidReplayInterpolation         = fmt.Errorf("%w: invalid replay interpolation", decaderr.ErrUnsupported)
	errReplayInvalidExactFraction               = fmt.Errorf("%w: invalid exact fraction", decaderr.ErrUnsupported)
)
