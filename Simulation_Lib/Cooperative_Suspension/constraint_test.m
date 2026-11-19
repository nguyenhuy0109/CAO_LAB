% Bước 1: Khai báo khoảng giá trị cho x và y
x = linspace(0, 10, 100);
y = linspace(2, 10, 100);

% Tạo lưới tọa độ 2D
[X, Y] = meshgrid(x, y);

% Bước 2: Tính giá trị hàm số Z theo X và Y
Z = X./(Y - 1);

% Bước 3: Vẽ đồ thị mặt 3D
figure;
surf(X, Y, Z); % Hoặc dùng mesh(X, Y, Z) nếu chỉ muốn xem dạng lưới khung

% Bước 4: Thêm các tùy chọn hiển thị và trang trí
title('Đồ thị mặt 3D của hàm z = sin(x)*cos(y)');
xlabel('Trục X');
ylabel('Trục Y');
zlabel('Trục Z');

% Làm mượt màu sắc (bỏ các đường lưới đen trên mặt)
shading interp; 

% Thêm thanh màu minh họa giá trị Z
colorbar;

% Xoay góc nhìn mặc định đẹp hơn (tùy chọn)
view(45, 30);
grid on;