package decad

import "fmt"

// OperationKind names a public modeling or selector-resolution call.
type OperationKind int

const (
	OperationUnknown OperationKind = iota
	OperationExtrude
	OperationRevolve
	OperationSweep
	OperationLoft
	OperationLoftChain
	OperationLoftSections
	OperationLoftFromPoint
	OperationPatchProfile
	OperationPatchEdges
	OperationCut
	OperationUnion
	OperationIntersect
	OperationTransform
	OperationCopy
	OperationPatternCopies
	OperationPatterned
	OperationPatternJoin
	OperationFillet
	OperationChamfer
	OperationShell
	OperationDraft
	OperationOffset
	OperationThicken
	OperationStitch
	OperationUnstitch
	OperationTrim
	OperationExtend
	OperationSplit
	OperationRib
	OperationWeb
	OperationConeFromFace
	OperationRemove
	OperationSelectEdges
	OperationSelectFaces
	OperationHole
	OperationEmboss
	OperationCoil
	OperationMirrored
	OperationMirroredCopy
	OperationScaledAt
	OperationLoftBetween
	OperationLoftFaces
	OperationLoftFacesToConeTangent
	OperationThread
	OperationBoss
	OperationSolidFromMesh
)

// String returns the stable lower-snake token for k.
func (k OperationKind) String() string {
	if int(k) >= 0 && int(k) < len(operationKindNames) {
		return operationKindNames[k]
	}
	return fmt.Sprintf("OperationKind(%d)", int(k))
}

var operationKindNames = [...]string{
	operationUnknownName,
	"extrude",
	"revolve",
	"sweep",
	"loft",
	"loft_chain",
	"loft_sections",
	"loft_from_point",
	"patch_profile",
	"patch_edges",
	"cut",
	"union",
	"intersect",
	"transform",
	"copy",
	"pattern_copies",
	"patterned",
	"pattern_join",
	"fillet",
	"chamfer",
	"shell",
	"draft",
	"offset",
	"thicken",
	"stitch",
	"unstitch",
	"trim",
	"extend",
	"split",
	"rib",
	"web",
	"cone_from_face",
	"remove",
	"select_edges",
	"select_faces",
	"hole",
	"emboss",
	"coil",
	"mirrored",
	"mirrored_copy",
	"scaled_at",
	"loft_between",
	"loft_faces",
	"loft_faces_to_cone_tangent",
	"thread",
	"boss",
	"solid_from_mesh",
}
