import json
from typing import List, Mapping, Optional, Tuple


class TmJsonToDiction:
    @staticmethod
    def json_to_dict(json_string: str) -> Mapping:
        json_array = json.loads(json_string)
        return_dict = {}
        for dic in json_array:
            return_dict.update({dic["Item"]: dic["Value"]})
        return return_dict

    @staticmethod
    def tm_string_to_json(string: str) -> str:
        pos = [pos for pos, char in enumerate(string) if char == ","]

        # new_string = string[:-4]
        new_string = string[: pos[len(pos) - 1]]
        for _ in range(4):
            comma_position = new_string.find(",")
            new_string = new_string[comma_position + 1 :]
        return new_string

    @staticmethod
    def split_package(string: str) -> Tuple[str, Optional[List[str]]]:
        header_position = [i for i in range(len(string)) if string.startswith("$TMSVR", i)]

        if len(header_position) == 0:
            return string, None
        star_position = [i for i in range(len(string)) if string.startswith("*", i)]
        if len(star_position) == 0:
            return string, None

        i = 0
        new_string = []
        remain_string = ""
        while i < len(star_position):
            if i == len(header_position):
                break
            string_start = header_position[i]
            if i + 1 >= len(header_position):
                string_end = len(string)
            else:
                if header_position[i + 1] < star_position[i]:
                    header_position.remove(header_position[i + 1])
                    continue
                string_end = header_position[i + 1]
            i += 1
            new_string.append(string[string_start:string_end])

        if string_end < len(string):
            remain_string = string[string_end:]
            return remain_string, new_string
        if -1 == new_string[len(new_string) - 1].find("*"):
            remain_string = new_string[len(new_string) - 1]
            del new_string[-1]

        return remain_string, new_string


def print_splited_string_and_nokori(new_string: List[str], nokori: str) -> None:
    if new_string is not None:
        for string in new_string:
            print(string)
    print("***nokori is***")
    print(nokori)


if __name__ == "__main__":
    input_str = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E'  # noqa: E501
    json_string = TmJsonToDiction.tm_string_to_json(input_str)
    print(json_string)
    dictionary = TmJsonToDiction.json_to_dict(json_string)
    print(dictionary)

    # <ok>               in test 4
    # <ok><ok>           in test 2
    # <ok><ok><nokori>   in test 7
    # <nokori>           in test 10
    # <error>            in test 8
    # <error><ok>        in test 5
    # <error><ok><ok>    in test 9
    # <error><ok><nokori>in test 6
    # <error><nokori>    in test 3

    print("------test 2----<ok><ok><nokori>------")
    input_str2 = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.37033'  # noqa: E501

    nokori, new_string = TmJsonToDiction.split_package(input_str2)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 3-----<error><nokori>-----")
    input_str3 = 'VR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value"'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str3)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 4----<ok>------")
    input_str4 = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str4)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 5----<error><ok>------")
    input_str5 = 'Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2B $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2B'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str5)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 6----<error><ok><nokori>------")
    input_str6 = 'Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2B $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str6)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 7----<ok><ok><nokori>------")
    input_str7 = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str7)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 8----<error>------")
    input_str8 = '228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Val'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str8)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 9----<error><ok><ok>------")
    input_str9 = 'R,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E'  # noqa: E501
    nokori, new_string = TmJsonToDiction.split_package(input_str9)
    print_splited_string_and_nokori(new_string, nokori)

    print("------test 10----<nokori>------")
    input_str10 = '$TMSVR,228,0,3,[{"Item":'
    nokori, new_string = TmJsonToDiction.split_package(input_str10)
    print_splited_string_and_nokori(new_string, nokori)
