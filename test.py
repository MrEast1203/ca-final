# # a = 0xb526de2f
# # b=0xb5478711
# # c=0x0000000080000000
# # a=format(a,'0>32b')
# # b=format(b,'0>32b')
# # c=format(c,'0>64b')
# # a='1111100000010101100010100100011'
# # b='1000010111010011000011000000111'
# a=0xb9a3ed49
# b=0xf6be79d5
# a=format(a,'0>32b')
# b=format(b,'0>32b')
# def flip(x):
#     x=bin(int(x,2)-1)
#     ans=''
#     for i in range(len(x)-2):
#         if x[i+2] == '0':
#             ans = ans + '1'
#         else: ans = ans + '0'
#     return ans
# def flip2(x):
#     ans=''
#     for i in range(len(x)):
#         if x[i] == '0':
#             ans = ans + '1'
#         else: ans = ans + '0'
#     x=bin(int(ans,2)+1)[2:]
#     return ans
# print(a,'is a',int(a[1:],2)-2**31)
# print(b,'is b',int(b[1:],2)-2**31)
# # a=(flip(a))
# # b=(flip(b))
# # print(a,'is fa')
# # print(b,'is fb')
# ans=-1180439223-155289131

a=0x00a2a223
print(format(a,'0>32b'))
# print("0000000-0000000000-000-00110-0010111")
# print(hex(int('10101110001010101010111100001000',2)))
def transform_string(input_string):
    a=input_string[0:7]
    b=input_string[7:17]
    c=input_string[17:20]
    d=input_string[20:25]
    e=input_string[25:32]
    return a+'-'+b+'-'+c+'-'+d+'-'+e

# Example usage
input_string = format(a,'0>32b')
transformed_string = transform_string(input_string)
print(transformed_string)
