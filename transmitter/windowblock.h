#include <stdint.h>
class WindowBlock
{
public:
	int index;
	int window_index; // Window的編號, 也是所在的檔案絕對位置 (FILE Position)
	unsigned char *data;  // Window的資料
	double overhead; // overhead
	int window_size;  // 實際的Window的大小 (Byte), default: 2M
	int block_size;  // 期望切割Window內的Message Block大小 (Byte), default: 1K
	int block_num; // 期望切割Window內中有幾個Message Block, default: 2000
	int *layer_index; // 各個Layer的起始絕對位置
	int *layer_size; // Layeri中的Message Block大小 n
	double *layer_weight; // 各個Layer被挑選中的機率
	int layer_num;  // Window內有幾個Layer, 在此只實作2層, default: 2
	int layer_last;
	int m; // m=0, 代表為最後的Window Block, m=1代表還有其它的Window Block
};