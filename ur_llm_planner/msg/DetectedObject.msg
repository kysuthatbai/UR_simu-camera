# Mot khoi duoc camera nhan dien (perception_node).
string name                      # ten vat, vd: red_cube
bool visible                     # true: thay trong khung hinh gan nhat
bool known                       # true: da tung thay it nhat mot lan
geometry_msgs/Point position     # tam khoi trong frame world (vi tri quan sat gan nhat)
string in_zone                   # ten vung chua khoi; "" neu khoi nam tren ban, ngoai moi vung
float32 pixel_area               # dien tich vung mau (pixel), de chan doan
builtin_interfaces/Time last_seen
