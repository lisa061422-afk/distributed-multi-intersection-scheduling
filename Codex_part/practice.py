from typing import List

# nums = [10, 20, 30]
# nums.append(4)
# nums.pop()
# nums[0]
# len(nums)

# for x in nums:
#     print(x)

# for i, x in enumerate(nums):
#     print(i,x)

# set 哈希集合：查有没有
# seen = set()

# seen.add(3)
# seen.add(5)
# print(seen)          # {3, 5}
# print(3 in seen)    # True
# print(4 in seen)    # False

# nums = [3, 5, 3, 2]
# for num in nums:
#     if num in seen:
#         print(num, "已经在 seen 里")
#     else:
#         print(num, "第一次见")
#         seen.add(num)

# dict 哈希表：记录 key -> value
# mp = {}

# mp["apple"] = 3
# mp["banana"] = 5

# print(mp)              # {'apple': 3, 'banana': 5}
# print(mp["apple"])     # 3

# print("apple" in mp)   # True
# print("orange" in mp)  # False

# def twoSum(nums, target): # this is input of a function
#     mp = {}
    
#     for i, num in enumerate(nums):
#         need = target - num

#         if need in mp:
#             return [mp[need], i]
        
#         mp[num] = i

# nums = [3, 4, 5, 6]
# target = 7

# print(twoSum(nums, target))  # [0, 1]

# def encode(strs: list[str]) -> str:
#         parts = []
#         for s in strs:
#             parts.append(str(len(s)))
#             parts.append("#")
#             parts.append(s) 

#         return "".join(parts)

# def decode(s: str) -> list[str]:
#         res = []
#         i = 0 #指针起点

#         while i < len(s):
#             j = i #初始化指针终点

#             while s[j] != "#":
#                 j += 1
            
#             length = int(s[i:j]) #[start,end) 不包括end这个位置的
#             start = j + 1
#             end = start + length

#             res.append(s[start:end])

#             i = end
#         return res

# x = encode(["hello","world"])
# print(x)
# y = decode(x)
# print(y)

def productExceptSelf(nums: List[int]) -> List[int]:
        n = len(nums)
        res = [1] * n

        prefix = 1
        for i in range(n):
            res[i] = prefix
            prefix *= nums[i]

        postfix = 1
        for i in range(n - 1, -1, -1):
            res[i] *= postfix
            postfix *= nums[i]

        return res

x = productExceptSelf([2,5,8])
print(x)