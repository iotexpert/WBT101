/***************************************************************************//**
* File Name: cycfg_bt.h
* Version: 1.1
*
* Description:
* Bluetooth Configurator configuration.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 1.1.0 build 291
*
********************************************************************************
* Copyright 2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if !defined(CYCFG_BT_H)
#define CYCFG_BT_H

/*
BLE_CONFIG_START
<?xml version="1.0"?>
<Configuration major="1" minor="1" device="20xxx">
    <Profiles>
        <Profile name="GATT">
            <ProfileRoles>
                <ProfileRole type="Server">
                    <ProfileRoleProperties>
                        <Property id="Name" value="Server"/>
                    </ProfileRoleProperties>
                    <Services>
                        <Service type="org.bluetooth.service.generic_access">
                            <ServiceProperties>
                                <Property id="EntityID" value="{cdf39c9f-ab15-4d51-b07b-f1c12fe29bc8}"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics>
                                <Characteristic type="org.bluetooth.characteristic.gap.device_name">
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="Name"/>
                                                <Property id="Value" value="change_me"/>
                                                <Property id="Format" value="f_utf8s"/>
                                                <Property id="ByteLength" value="9"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="true"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="false"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors/>
                                </Characteristic>
                                <Characteristic type="org.bluetooth.characteristic.gap.appearance">
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="Category"/>
                                                <Property id="EnumValue" value="0"/>
                                                <Property id="Format" value="f_16bit"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="true"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="false"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors/>
                                </Characteristic>
                            </Characteristics>
                        </Service>
                        <Service type="org.bluetooth.service.generic_attribute">
                            <ServiceProperties>
                                <Property id="EntityID" value="{7eb23768-b02b-4651-9ac3-cfa781caa922}"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics/>
                        </Service>
                        <Service type="org.bluetooth.service.custom">
                            <ServiceProperties>
                                <Property id="EntityID" value="{e2ce2beb-7e2b-49ca-9aea-9866eca5938f}"/>
                                <Property id="DisplayName" value="Modus"/>
                                <Property id="UUID" value="D2FF33E6-CA5F-4C4B-A34C-CC928D996AA7"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics>
                                <Characteristic type="org.bluetooth.characteristic.custom">
                                    <CharacteristicProperties>
                                        <Property id="DisplayName" value="Counter"/>
                                        <Property id="UUID" value="A50C0D99-2581-4FB3-A760-53E36D1203D2"/>
                                    </CharacteristicProperties>
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="New field"/>
                                                <Property id="Value" value="0"/>
                                                <Property id="Format" value="f_uint8"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WriteWithoutResponse"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="AuthenticatedSignedWrites"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="ReliableWrite"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Notify"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Indicate"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WritableAuxiliaries"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Broadcast"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="false"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors>
                                        <Descriptor type="org.bluetooth.descriptor.gatt.client_characteristic_configuration">
                                            <Fields>
                                                <Field>
                                                    <FieldProperties>
                                                        <Property id="Name" value="Properties"/>
                                                        <Property id="Value" value=""/>
                                                        <Property id="Format" value="f_16bit"/>
                                                    </FieldProperties>
                                                    <BitField>
                                                        <Property id="BitValue" value="0"/>
                                                        <Property id="BitValue" value="0"/>
                                                    </BitField>
                                                </Field>
                                            </Fields>
                                            <Properties>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Read"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Write"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                            </Properties>
                                            <Permission>
                                                <Property id="Read" value="true"/>
                                                <Property id="ReadAuthenticated" value="false"/>
                                                <Property id="VariableLength" value="false"/>
                                                <Property id="Write" value="true"/>
                                                <Property id="WriteNoResponse" value="false"/>
                                                <Property id="WriteAuthenticated" value="false"/>
                                                <Property id="WriteReliable" value="false"/>
                                            </Permission>
                                        </Descriptor>
                                        <Descriptor type="org.bluetooth.descriptor.gatt.characteristic_user_description">
                                            <Fields>
                                                <Field>
                                                    <FieldProperties>
                                                        <Property id="Name" value="User Description"/>
                                                        <Property id="Value" value="Count of button presses"/>
                                                        <Property id="Format" value="f_utf8s"/>
                                                        <Property id="ByteLength" value="23"/>
                                                    </FieldProperties>
                                                </Field>
                                            </Fields>
                                            <Properties>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Read"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Write"/>
                                                    <Property id="Present" value="false"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                            </Properties>
                                            <Permission>
                                                <Property id="Read" value="true"/>
                                                <Property id="ReadAuthenticated" value="false"/>
                                                <Property id="VariableLength" value="false"/>
                                                <Property id="Write" value="false"/>
                                                <Property id="WriteNoResponse" value="false"/>
                                                <Property id="WriteAuthenticated" value="false"/>
                                                <Property id="WriteReliable" value="false"/>
                                            </Permission>
                                        </Descriptor>
                                    </Descriptors>
                                </Characteristic>
                            </Characteristics>
                        </Service>
                        <Service type="org.bluetooth.service.custom">
                            <ServiceProperties>
                                <Property id="EntityID" value="{7837480d-cd67-4c52-b342-a6760d0f1996}"/>
                                <Property id="DisplayName" value="OTA_FW_UPGRADE"/>
                                <Property id="UUID" value="AE5D1E47-5C13-43A0-8635-82AD38A1381F"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics>
                                <Characteristic type="org.bluetooth.characteristic.custom">
                                    <CharacteristicProperties>
                                        <Property id="DisplayName" value="CONTROL_POINT"/>
                                        <Property id="UUID" value="A3DD50BF-F7A7-4E99-838E-570A086C661B"/>
                                    </CharacteristicProperties>
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="New field"/>
                                                <Property id="Value" value=""/>
                                                <Property id="Format" value="f_utf8s"/>
                                                <Property id="ByteLength" value="0"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WriteWithoutResponse"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="AuthenticatedSignedWrites"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="ReliableWrite"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Notify"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Indicate"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WritableAuxiliaries"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Broadcast"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="false"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="true"/>
                                        <Property id="Write" value="true"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors>
                                        <Descriptor type="org.bluetooth.descriptor.gatt.client_characteristic_configuration">
                                            <Fields>
                                                <Field>
                                                    <FieldProperties>
                                                        <Property id="Name" value="Properties"/>
                                                        <Property id="Value" value=""/>
                                                        <Property id="Format" value="f_16bit"/>
                                                    </FieldProperties>
                                                    <BitField>
                                                        <Property id="BitValue" value="0"/>
                                                        <Property id="BitValue" value="0"/>
                                                    </BitField>
                                                </Field>
                                            </Fields>
                                            <Properties>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Read"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Write"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                            </Properties>
                                            <Permission>
                                                <Property id="Read" value="true"/>
                                                <Property id="ReadAuthenticated" value="false"/>
                                                <Property id="VariableLength" value="false"/>
                                                <Property id="Write" value="true"/>
                                                <Property id="WriteNoResponse" value="false"/>
                                                <Property id="WriteAuthenticated" value="false"/>
                                                <Property id="WriteReliable" value="false"/>
                                            </Permission>
                                        </Descriptor>
                                    </Descriptors>
                                </Characteristic>
                                <Characteristic type="org.bluetooth.characteristic.custom">
                                    <CharacteristicProperties>
                                        <Property id="DisplayName" value="DATA"/>
                                        <Property id="UUID" value="A2E86C7A-D961-4091-B74F-2409E72EFE26"/>
                                    </CharacteristicProperties>
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="New field"/>
                                                <Property id="Value" value=""/>
                                                <Property id="Format" value="f_utf8s"/>
                                                <Property id="ByteLength" value="0"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WriteWithoutResponse"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="AuthenticatedSignedWrites"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="ReliableWrite"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Notify"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Indicate"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WritableAuxiliaries"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Broadcast"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="false"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="true"/>
                                        <Property id="Write" value="true"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors/>
                                </Characteristic>
                            </Characteristics>
                        </Service>
                    </Services>
                </ProfileRole>
            </ProfileRoles>
        </Profile>
    </Profiles>
</Configuration>
BLE_CONFIG_END
*/

#endif /* CYCFG_BT_H */

/* [] END OF FILE */

