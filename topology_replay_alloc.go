package decad

import "unsafe"

// ReplayNodeSize reports the storage for the handle containing each topology node.
func (handleAlloc) ReplayNodeSize(kind string) (uint64, bool) {
	switch kind {
	case "body":
		return uint64(unsafe.Sizeof(Body{})), true
	case "lump":
		return uint64(unsafe.Sizeof(Lump{})), true
	case "shell":
		return uint64(unsafe.Sizeof(FaceShell{})), true
	case "face":
		return uint64(unsafe.Sizeof(Face{})), true
	case "loop":
		return uint64(unsafe.Sizeof(Loop{})), true
	case "edge":
		return uint64(unsafe.Sizeof(Edge{})), true
	case "vertex":
		return uint64(unsafe.Sizeof(Vertex{})), true
	default:
		return 0, false
	}
}
