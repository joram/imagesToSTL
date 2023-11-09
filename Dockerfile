FROM gocv/opencv:4.8.1 as builder
WORKDIR /app

# Install dependencies
ADD go.mod .
RUN go mod download

# Build
ADD . .
RUN go build -o bin/server -v

FROM alpine:3.14
WORKDIR /app
COPY --from=builder /app/server /app/server
RUN chmod +x /app/server
CMD ["/app/server"]