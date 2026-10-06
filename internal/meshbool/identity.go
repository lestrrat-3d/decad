package meshbool

import (
	"fmt"
)

// OperationKind is the evaluator's private operation vocabulary.
type OperationKind int

const (
	OpUnion OperationKind = iota
	OpCut
	OpIntersect
)

func (k OperationKind) String() string {
	switch k {
	case OpUnion:
		return "union"
	case OpCut:
		return "cut"
	case OpIntersect:
		return "intersect"
	default:
		return fmt.Sprintf("operationKind(%d)", int(k))
	}
}
