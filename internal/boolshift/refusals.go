package boolshift

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

var (
	ErrPiecesNeedMeshAndBudget       = fmt.Errorf("%w: bounded pieces need a mesh and budget", decaderr.ErrUnsupported)
	ErrInvalidMeshIndex              = fmt.Errorf("%w: invalid bounded mesh index", decaderr.ErrBooleanFailed)
	ErrInvalidReplayShifts           = fmt.Errorf("%w: invalid replay shifts", decaderr.ErrUnsupported)
	ErrReplayShiftCapacity           = fmt.Errorf("%w: replay shift capacity", decaderr.ErrUnsupported)
	ErrReplayCylinderImageDiffers    = fmt.Errorf("%w: replay cylinder image differs", decaderr.ErrUnsupported)
	ErrReplayMovedTagTypeUnavailable = fmt.Errorf("%w: replay moved tag type unavailable", decaderr.ErrUnsupported)
	ErrReplayPlaneImageDiffers       = fmt.Errorf("%w: replay plane image differs", decaderr.ErrUnsupported)
	ErrUnsupportedReplayMovedGroup   = fmt.Errorf("%w: unsupported replay moved group", decaderr.ErrUnsupported)
)

var ErrShiftDeclined = errors.New("booleaneval: shifted retry declined")
