// Package payloadclass lets the public test kit (decadtest) read which payload
// class built a body. The root package installs Reader when it loads; the kit
// calls Of. Nothing in the library reads it, and it adds no public API.
package payloadclass

// Reader returns the payload type name of a *decad.Body, as
// reportvocab.PayloadTypeName prints it, or "" for any other value. The root
// package sets it once at load time.
var Reader func(body any) string

// Of returns the payload class name of body, or "" before the root package has
// installed Reader or when body is not a body handle.
func Of(body any) string {
	if Reader == nil {
		return ""
	}
	return Reader(body)
}
