package booleaneval

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These immutable authentication refusals are initialized before replay starts.
var (
	errShiftAuthGrownCutCertificateIsMissing            = fmt.Errorf("%w: grown Cut certificate is missing", decaderr.ErrUnsupported)
	errShiftAuthInvalidPrivateShiftEnclosure            = fmt.Errorf("%w: invalid private shift enclosure", decaderr.ErrUnsupported)
	errShiftAuthMissingReplayBoxMeter                   = fmt.Errorf("%w: missing replay box meter", decaderr.ErrUnsupported)
	errShiftAuthMissingReplayMesh                       = fmt.Errorf("%w: missing replay mesh", decaderr.ErrUnsupported)
	errShiftAuthNoFiniteReplayEnclosure                 = fmt.Errorf("%w: no finite replay enclosure", decaderr.ErrUnsupported)
	errShiftAuthNonfiniteReplayVertex                   = fmt.Errorf("%w: nonfinite replay vertex", decaderr.ErrUnsupported)
	errShiftAuthPrivateShiftEvidenceMissing             = fmt.Errorf("%w: private shift evidence missing", decaderr.ErrUnsupported)
	errShiftAuthPrivateShiftExceedsItsPermittedReach    = fmt.Errorf("%w: private shift exceeds its permitted reach", decaderr.ErrUnsupported)
	errShiftAuthPrivateShiftGateChanged                 = fmt.Errorf("%w: private shift gate changed", decaderr.ErrUnsupported)
	errShiftAuthPrivateShiftIsNotPositivePureGrowth     = fmt.Errorf("%w: private shift is not positive pure growth", decaderr.ErrUnsupported)
	errShiftAuthPrivateShiftPairChordChanged            = fmt.Errorf("%w: private shift pair chord changed", decaderr.ErrUnsupported)
	errShiftAuthPrivateShiftWarningChanged              = fmt.Errorf("%w: private shift warning changed", decaderr.ErrUnsupported)
	errShiftAuthRetainedCutTargetIsUnavailable          = fmt.Errorf("%w: retained Cut target is unavailable", decaderr.ErrUnsupported)
	errShiftAuthRetainedMeshExceedsRecordLimit          = fmt.Errorf("%w: retained mesh exceeds record limit", decaderr.ErrUnsupported)
	errShiftAuthShiftCertificateIdentityOrSourceChanged = fmt.Errorf("%w: shift certificate identity or source changed", decaderr.ErrUnsupported)
	errShiftAuthShiftCertificateIsNotATranslation       = fmt.Errorf("%w: shift certificate is not a translation", decaderr.ErrUnsupported)
	errShiftAuthShiftCertificateReplayChanged           = fmt.Errorf("%w: shift certificate replay changed", decaderr.ErrUnsupported)
	errShiftAuthShiftCertificateSourceChanged           = fmt.Errorf("%w: shift certificate source changed", decaderr.ErrUnsupported)
	errShiftAuthShiftGeometryChanged                    = fmt.Errorf("%w: shift geometry changed", decaderr.ErrUnsupported)
	errShiftAuthShiftedCutCertificateIsMissing          = fmt.Errorf("%w: shifted Cut certificate is missing", decaderr.ErrUnsupported)
	errShiftAuthShiftedCutWarningOrChordChanged         = fmt.Errorf("%w: shifted Cut warning or chord changed", decaderr.ErrUnsupported)
	errShiftAuthShiftedMeshBoundDecreased               = fmt.Errorf("%w: shifted mesh bound decreased", decaderr.ErrUnsupported)
	errShiftAuthShiftedToolIsMissing                    = fmt.Errorf("%w: shifted tool is missing", decaderr.ErrUnsupported)
	errShiftAuthStoredCutWarningChanged                 = fmt.Errorf("%w: stored Cut warning changed", decaderr.ErrUnsupported)
)
