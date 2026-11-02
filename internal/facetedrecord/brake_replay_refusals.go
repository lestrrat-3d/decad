package facetedrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable refusals exist before the replay budget starts.
var (
	errBrakeReplayAFacetMapsToNoFace                                           = fmt.Errorf("%w: a facet maps to no face", decaderr.ErrBooleanFailed)
	errBrakeReplayAFacetedPayloadSMeshBoundIsNotTheLargestOfItsFacetBounds     = fmt.Errorf("%w: a faceted payload's mesh bound is not the largest of its facet bounds", decaderr.ErrBooleanFailed)
	errBrakeReplayAFacetedPayloadSPerVertexOrPerFacetRecordDoesNotMatchItsMesh = fmt.Errorf("%w: a faceted payload's per-vertex or per-facet record does not match its mesh", decaderr.ErrBooleanFailed)
	errBrakeReplayBoundedCylinderImageDiffers                                  = fmt.Errorf("%w: bounded cylinder image differs", decaderr.ErrUnsupported)
	errBrakeReplayBoundedCylinderOriginIsInexact                               = fmt.Errorf("%w: bounded cylinder origin is inexact", decaderr.ErrUnsupported)
	errBrakeReplayBoundedCylinderOriginRange                                   = fmt.Errorf("%w: bounded cylinder origin range", decaderr.ErrUnsupported)
	errBrakeReplayBoundedCylinderRadiusIsInvalid                               = fmt.Errorf("%w: bounded cylinder radius is invalid", decaderr.ErrUnsupported)
	errBrakeReplayBoundedCylinderVertexRange                                   = fmt.Errorf("%w: bounded cylinder vertex range", decaderr.ErrUnsupported)
	errBrakeReplayBoundedPlanePlacementRequiresTranslation                     = fmt.Errorf("%w: bounded plane placement requires translation", decaderr.ErrUnsupported)
	errBrakeReplayBoundedRestatementRequiresABodyAndBudget                     = fmt.Errorf("%w: bounded restatement requires a body and budget", decaderr.ErrBooleanFailed)
	errBrakeReplayInvalidBoundedSourceDiameter                                 = fmt.Errorf("%w: invalid bounded source diameter", decaderr.ErrUnsupported)
	errBrakeReplayInvalidFacetVertexInFacetedRestatement                       = fmt.Errorf("%w: invalid facet vertex in faceted restatement", decaderr.ErrBooleanFailed)
	errBrakeReplayInvalidShellInFacetedRestatement                             = fmt.Errorf("%w: invalid shell in faceted restatement", decaderr.ErrBooleanFailed)
	errBrakeReplayInvalidSourceVertex                                          = fmt.Errorf("%w: invalid source vertex", decaderr.ErrUnsupported)
	errBrakeReplayMissingReplayTopologyMeter                                   = fmt.Errorf("%w: missing replay topology meter", decaderr.ErrUnsupported)
	errBrakeReplayNilLumpInFacetedRestatement                                  = fmt.Errorf("%w: nil lump in faceted restatement", decaderr.ErrBooleanFailed)
	errBrakeReplayReplayLoopEdgeAbsent                                         = fmt.Errorf("%w: replay loop edge absent", decaderr.ErrBooleanFailed)
	errBrakeReplayReplayLoopFaceAbsent                                         = fmt.Errorf("%w: replay loop face absent", decaderr.ErrBooleanFailed)
	errBrakeReplayReplayRetainedCurveChainExceedsWorkLimit                     = fmt.Errorf("%w: replay retained-curve chain exceeds work limit", decaderr.ErrUnsupported)
	errBrakeReplaySignedCylinderRadiusDiffers                                  = fmt.Errorf("%w: signed cylinder radius differs", decaderr.ErrUnsupported)
	errBrakeReplaySignedPlacementImageIsRounded                                = fmt.Errorf("%w: signed placement image is rounded", decaderr.ErrUnsupported)
	errBrakeReplaySignedPlacementUnavailable                                   = fmt.Errorf("%w: signed placement unavailable", decaderr.ErrUnsupported)
	errBrakeReplayTranslatedCylinderNormalBoundUnavailable                     = fmt.Errorf("%w: translated cylinder normal bound unavailable", decaderr.ErrUnsupported)
	errBrakeReplayTranslatedPlaneDirectionsDiffer                              = fmt.Errorf("%w: translated plane directions differ", decaderr.ErrUnsupported)
	errBrakeReplayTranslatedPlaneFrameDiffers                                  = fmt.Errorf("%w: translated plane frame differs", decaderr.ErrUnsupported)
	errBrakeReplayUnsupportedBoundedCylinderPlacement                          = fmt.Errorf("%w: unsupported bounded cylinder placement", decaderr.ErrUnsupported)
)
