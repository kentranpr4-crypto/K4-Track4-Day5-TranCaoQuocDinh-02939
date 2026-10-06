# Kết quả kiểm thử UWB mô phỏng

Ngày chạy: 2026-10-06. Không có dữ liệu UWB phần cứng trong repository.
RMSE dưới đây là trung bình RMSE vị trí Euclid 2D của 10 seed (0–9), đơn vị mét.
Tham số cố định: sigma=0.3 m/trục, q=0.5 m²/s³, gate cap=1 m, gate 0.999.
Skip 3 giây khởi động; các phương pháp dùng cùng timestamp có raw hợp lệ.
Truth chỉ dùng chấm điểm. Bộ kịch bản đã tham gia kiểm tra/sửa thiết kế, không phải tập thực địa độc lập.

| Kịch bản | Raw | MA 7 | Kalman | Robust online | RTS offline |
|---|---:|---:|---:|---:|---:|
| Đứng yên | 0.425 | 0.159 | 0.213 | 0.213 | 0.113 |
| Chuyển động cong | 0.425 | 0.267 | 0.214 | 0.213 | 0.113 |
| Cua vuông và dừng | 0.425 | 0.270 | 0.215 | 0.214 | 0.114 |
| Spike + burst | 2.251 | 0.847 | 1.099 | 0.246 | 0.121 |
| Spike + burst + dropout (ngoài mất mẫu) | 2.267 | 0.854 | 1.107 | 0.247 | 0.121 |
| Bias kéo dài | 1.809 | 1.756 | 1.770 | 1.755 | 1.748 |
| Chỉ thời gian mất mẫu | — | — | 2.206 | 0.700 | 0.210 |

- Với spike/burst: raw 2.251 m → online 0.246 m (~89% giảm RMSE), offline 0.121 m (~95%).
- Đứng yên: MA 7 tốt hơn cấu hình online dành cho vật di động; RTS đạt 0.113 m.
- Bias kéo dài: vẫn sai khoảng 1.75 m sau lọc. Không suy ra hết bias từ một đường mượt.
- Khi mất mẫu: online chỉ ngoại suy (0.700 m), offline dùng điểm tương lai (0.210 m).
- P95 trong JSON là trung bình P95 từng seed, không phải percentile gộp toàn bộ mẫu.
- Không đo trễ thiết bị hay trễ pha riêng; lỗi vị trí được chấm ở timestamp đồng bộ.

## Kiểm tra đã chạy

Notebook hiện đã được cá nhân hóa cho **02939**; xem [báo cáo hiện tại](LAB_02939.md).
Các số Phần 9 ghi “demo” bên dưới là kết quả lịch sử với `uwb_demo`, không phải của 02939.

- 11/11 unit test pass, gồm kiểm tra tính nhân quả và seed riêng 101/202/303.
- Toàn bộ 60 cell code của notebook chạy thành công; các check bắt buộc và EKF đều pass.
- Demo Phần 9: GPS mean/median NIS 32.55/1.61; UWB 3.24/1.46. Gate GPS cho pooled mean NIS sau chọn lọc 1.994, nhận 1311/1350 mẫu.
- RMS độ bất định vị trí cuối của demo Phần 9: 0.457 m; bán kính đường tròn bao elip Gaussian 95%: 0.790 m. Không có reference công khai để khẳng định RMSE của nhiệm vụ này.
- CLI đọc 600 mẫu example_uwb.csv, xuất 600 dòng có online/offline, nhận 517 mẫu; các mẫu còn lại được giữ với cờ missing/rejected.

## Tái lập

```bash
python -m unittest discover -s tests -v
MPLCONFIGDIR=/tmp/uwb-mpl python scripts/benchmark_uwb.py
python -m uwb_filter reports/example_uwb.csv --output reports/example_filtered.csv --smooth
MPLCONFIGDIR=/tmp/uwb-mpl python scripts/run_notebook.py
```

Môi trường đã chạy: Python 3.10.12, NumPy 2.2.6, SciPy 1.15.3, Matplotlib 3.10.8.
Môi trường có cảnh báo module Axes3D; các biểu đồ của project là 2D và đã xuất thành công.

![So sánh quỹ đạo, sai số, NIS và độ bất định](uwb_comparison.png)

Chi tiết từng seed: [benchmark.csv](benchmark.csv). Thống kê: [benchmark_summary.json](benchmark_summary.json).
