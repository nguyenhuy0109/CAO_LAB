# Overview

## Yêu cầu bài toán

```mermaid
flowchart TD
    PAYLOAD(Nâng tải) --> LOGISTIC
    COOPERATIVE(Điều khiển hợp tác) --> LOGISTIC
    LOGISTIC(Vận chuyển) --> FORMATION(Điều khiển đội hình)
    RECONFIG(Tái cấu trúc đội hình) --> FORMATION
    PATH_PLANNING(Tự lập kế hoạch đường đi) --> FORMATION
    COLISION_AVOIDANCE(Tránh va chạm nội bộ) --> PATH_PLANNING
    COLISION_AVOIDANCE1(Tránh va chạm địa hình) --> PATH_PLANNING
    CONNECTION(Giữ liên kết thông tin) --> PATH_PLANNING

    FORMATION --> ALGORITHM(Thuật toán điều khiển)
    FORMATION --> AI(Nhận diện ảnh, xử lí địa hình)

    ALGORITHM --> GRAPH_THEORY(Lí thuyết đồ thị)
    ALGORITHM --> CONSENSUS(Điều khiển đồng thuận)
    ALGORITHM --> FORM(Điều khiển đội hình)
    
    AI --> CAMERA(Nhận diện bằng camera)

    CAMERA --> DRONE(Nhận diện Drone)
    CAMERA --> VECTOR(Xác định hướng)

    AI --> LIDAR(Quét địa hình bằng LiDar)

    LIDAR --> DEPTH(Đo độ sâu, hình dạng địa hình + vật cản)

    linkStyle default stroke:red,stroke-width:3px
    
```

## Chia nhiệm vụ từng nhóm

### Nhóm thuật toán điều khiển đội hình
- Thiết kế thuật toán trên lí thuyết sử dụng các thuật toán điều khiển bền vững, dự báo, mạng noron,...
- Thiết kế trình mô phỏng, kiểm thử thuật toán

### Nhóm AI, xử lí ảnh
- Sử dụng, tùy chỉnh các mô hình nhận diện ảnh có sẵn để nhận diện Drone, con người
- Thực hiện thử nghiệm trên máy tính nhúng
- Tìm hiểu, triển các mô hình TinyML trên vi điều khiển ESP32

---

# Các công cụ thiết kế + mô phỏng
## Mô phỏng + kiểm thử thuật toán, công thức
- Matlab/Simulink
- C/C++
- Python

## Nhúng thuật toán + mô phỏng trong môi trường thực tế ảo
### NVIDIA Iaac Sim (Issac Lab / Pegasus Simulator)

### MuJoCo (MuJoCo MJX)

### Chuyên dụng cho hàng không (UAV)
- Flightmare (Đồ họa đẹp, tốc độ cao)
- X-Plane / FlightGear (Mô phỏng khí động học thực tế)
- RotorTM (Mô phỏng UAV bầy đàn mang tải treo dây cáp)
